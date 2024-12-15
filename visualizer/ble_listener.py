import asyncio
import struct

from bleak import (
    BleakScanner, 
    BleakClient, 
    BleakGATTCharacteristic
)

from thread_safe_queue import ThreadSafeQueue
from dataclasses import dataclass, field
from enum import IntEnum

class BoatState(IntEnum):
    BOAT_IDLE = 0
    BOAT_DRIVING = 1
    BOAT_ANCHORED = 2
    BOAT_ERROR = 3

class Direction(IntEnum):
    CW = -1
    NULL = 0
    CCW = 1

    @staticmethod
    def from_u8(u8: int) -> "Direction":
        return Direction(u8 - 1)
    
@dataclass
class MotorState:
    duty_cycle: int
    timeout: bool
    is_alive: bool
    is_idle: bool
    direction: Direction
    # reserved

@dataclass
class MotorboatPacket:
    boat_state: BoatState
    control_active: bool
    collision_detected: bool
    depth_exceeded: bool
    depth_exceeded: bool
    anchor_lifted: bool
    # reserved
    depth: int
    roll: int
    pitch: int
    timestamp: int
    motor_states: list[MotorState]

class BoatCommandType(IntEnum):
    RECOVERY_REQUEST = 0
    ANCHOR_BOAT = 1

class MotorboatBLEListener:
    def __init__(self, device_name: str, service_desc: str, tx_property: str, rx_property: str, address: str = None):
        self._name = device_name
        self._service_desc = service_desc
        self._tx_property = tx_property
        self._rx_property = rx_property
        
        self._addr: str = address
        self._data_queue: ThreadSafeQueue[MotorboatPacket] = ThreadSafeQueue(maxsize = 30)
        
        self._client: BleakClient
        self._tx_uuid: str
        self._rx_uuid: str
        
    def _deserialize_data(self, data: bytearray) -> MotorboatPacket:
        """
        Deserialization based upon the following structure:
        
        BoatState_t boat_state: 2;
        bool control_active: 1;
        bool collision_detected: 1;
        bool depth_exceeded: 1;
        bool anchor_lifted: 1;
        uint8_t reserved : 2; // for alignment                       // 1 byte
        uint16_t depth_mm;                                           // 2 bytes
        int8_t tilt_roll;                                            // 1 byte
        int8_t tilt_pitch;                                           // 1 byte
        uint32_t tick;                                               // 4 bytes
        MotorStatus_t motor_statuses[NUM_MOTORS + NUM_SPARE_MOTORS]; // 10 bytes
        """               
        print(data) 
        packet = MotorboatPacket(
            boat_state = BoatState(data[0] & 0x03),
            control_active = (data[0] >> 2) & 0x1,
            collision_detected = (data[0] >> 3) & 0x1,
            depth_exceeded = (data[0] >> 4) & 0x1,
            anchor_lifted = (data[0] >> 5) & 0x1,
            depth = struct.unpack('H', data[1:3]),
            roll = struct.unpack('b', bytes([data[3]])),
            pitch = struct.unpack('b', bytes([data[4]])),
            timestamp = struct.unpack('L', data[5:9]),
            motor_states = [
                MotorState(
                    duty_cycle = (data[i] & 0x7F),
                    timeout = (data[i] >> 7) & 0x1,
                    is_alive = data[i+1] & 0x1,
                    is_idle = (data[i+1] >> 1) & 0x1,
                    direction = Direction.from_u8((data[i+1] >> 2) & 0x3)
                ) for i in range(9, len(data)-1, 2)
            ]
        )
            
        return packet
        
    def notification_callback(self, _, data: bytearray):
        deserialized_data = self._deserialize_data(data)
        if deserialized_data is None:
            raise BufferError("Invalid data received")
        
        print(deserialized_data)
        self._data_queue.put(deserialized_data)

    async def initialize(self, timeout: int = 5) -> bool:
        if self._addr is None:
            if not await self.find_device(timeout): 
                print("Failed to find the device")
                return False
        else:
            print(f"Using address {self._addr}")
            
        # Connect to the client
        self._client = BleakClient(self._addr, timeout = timeout)
        await self._client.connect()
        
        if not await self.get_char_uuids():
            print("Failed to get the characteristic UUID's")
            return False
    
        print(f"{self._tx_uuid} {self._rx_uuid}")
        await self.dispatch_notification_listener()
        await self.dispatch_writer()
        while True:
            await asyncio.sleep(1)
    
    async def find_device(self, timeout: int) -> bool:
        print("Scanning for devices")
        devices = await BleakScanner.discover(timeout = timeout)
        for device in devices:
            if device.name == self._name:
                print(f"Device found: {device.address}!")
                self._addr = device.address
                
                return True
            
        return False
        
    async def get_char_uuids(self) -> bool:
        for service in self._client.services:
            if service.description == self._service_desc:
                for char in service.characteristics:
                    for property in char.properties:
                        if property == self._tx_property:
                            self._tx_uuid = char
                        elif property == self._rx_property:
                            self._rx_uuid = char
                
        print(f"{self._tx_uuid} {self._rx_uuid}")
        
        # Return if we initialized the UUID's for the Tx and Rx characteristics
        return self._tx_uuid is not None and self._rx_uuid is not None
    
    async def dispatch_notification_listener(self):
        # Start notification handler
        await self._client.start_notify(self._rx_uuid, self.notification_callback)

    async def dispatch_writer(self):
        # raw: 3630D31E1
        # raw: 000400000

        cmd_motion = struct.pack('B', 0) + struct.pack('H', 12342) + struct.pack('H', 7891)
        cmd_status = struct.pack('B', 1) + struct.pack('>I', (1 << 24) | 0x0)

        c = 0
        while True:
            c += 1
            if c % 2 == 0:
                await self._client.write_gatt_char(self._tx_uuid, cmd_motion)
            else:
                await self._client.write_gatt_char(self._tx_uuid, cmd_status)
            await asyncio.sleep(1)

def main():
    ble_listener = MotorboatBLEListener(
        device_name = "BlueNRG_Chat",
        service_desc = "Unknown",
        tx_property = "write",
        rx_property = "notify",
        address = "02:80:E1:00:00:AA"
    )
    
    asyncio.run(ble_listener.initialize(timeout = 30))
    
if __name__ == "__main__":
    main()
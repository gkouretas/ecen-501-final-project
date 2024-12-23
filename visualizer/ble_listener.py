import asyncio
import struct
import numpy as np
import queue

from bleak import (
    BleakScanner, 
    BleakClient
)

from bleak.exc import BleakDeviceNotFoundError

from thread_safe_queue import ThreadSafeQueue
from dataclasses import dataclass
from enum import IntEnum
import logging

logging.basicConfig(level=logging.DEBUG)

class BoatCommandType(IntEnum):
    COMMAND_MOTION = 0
    COMMAND_STATE = 1

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
    def from_u2(u2: int) -> "Direction":
        return Direction(u2 - 1)
    
class MotorType(IntEnum):
    NULL = -1 
    STEERING = 0
    THRUST = 1
    SPARE = 2
    
@dataclass
class MotorState:
    is_active: bool
    is_alive: bool
    is_idle: bool
    motor_type: MotorType

@dataclass
class MotorStatus:
    duty_cycle: int
    motor_state: MotorState
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
    motor_states: list[MotorStatus]
    temperature: int

class BoatRequestedCommand(IntEnum):
    RECOVERY_REQUEST = 0
    ANCHOR_BOAT = 1
    LIFT_ANCHOR = 2
    SWAP_MOTOR = 3

class MotorboatBLEListener:
    def __init__(self, device_name: str, service_desc: str, tx_property: str, rx_property: str, address: str = None):
        self._name = device_name
        self._service_desc = service_desc
        self._tx_property = tx_property
        self._rx_property = rx_property
        
        self._addr: str = address
        self._tx_queue_state: ThreadSafeQueue[bytes] = ThreadSafeQueue(maxsize = -1)
        self._tx_queue_cmd: ThreadSafeQueue[bytes] = ThreadSafeQueue(maxsize = 1)
        self._rx_queue: ThreadSafeQueue[MotorboatPacket] = ThreadSafeQueue(maxsize = 30)
        
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
        packet = MotorboatPacket(
            boat_state = BoatState(data[0] & 0x03),
            control_active = (data[0] >> 2) & 0x1,
            collision_detected = (data[0] >> 3) & 0x1,
            depth_exceeded = (data[0] >> 4) & 0x1,
            anchor_lifted = (data[0] >> 5) & 0x1,
            depth = struct.unpack('H', data[1:3])[0],
            roll = struct.unpack('b', bytes([data[3]]))[0],
            pitch = struct.unpack('b', bytes([data[4]]))[0],
            timestamp = struct.unpack('L', data[5:9])[0],
            motor_states = [
                MotorStatus(
                    duty_cycle = (data[i] & 0x7F),
                    motor_state = MotorState(
                        is_active = (data[i] >> 7) & 0x1,
                        is_alive = data[i+1] & 0x1,
                        is_idle = (data[i+1] >> 1) & 0x1,
                        motor_type = ((data[i+1] >> 4) & 0x3)
                    ),
                    direction = Direction.from_u2((data[i+1] >> 2) & 0x3)
                ) for i in range(9, len(data)-1, 2)
            ],
            temperature = data[19]
        )
            
        return packet
        
    def _process_data(self, data: bytearray):
        # TODO: this is too intensive and causes disconnects
        # need to dispatch elsewhere...
        deserialized_data = self._deserialize_data(data)
        if deserialized_data is None:
            raise BufferError("Invalid data received")
        try:
            self._rx_queue.put_nowait(deserialized_data)
        except queue.Full:
            print("Rx queue full, ignoring")
            return
        
    async def notification_callback(self, _, data: bytearray):
        self._process_data(data)
        
    def send_boat_request(self, request: BoatRequestedCommand):
        packet = struct.pack('B', BoatCommandType.COMMAND_STATE) + \
                struct.pack('>I', (request << 24) | 0x0)
        try:
            self._tx_queue_state.put_nowait(packet)
        except queue.Full:
            print(f"Unable to send command")
            return False
        
        return True
        
    def send_boat_motion_command(self, angle_rad: float, speed_percentage: int):
        angle = int(angle_rad * 32767 / (np.pi/2))
        speed = int(speed_percentage * 65535 / 100)
        packet = struct.pack('B', BoatCommandType.COMMAND_MOTION) + \
                struct.pack('h', angle) + \
                struct.pack('H', speed)
                
        try:
            self._tx_queue_cmd.put_nowait(packet)
        except queue.Full:
            return False
        
        return True

    def pop_command(self) -> MotorboatPacket:
        return self._rx_queue.get_nowait()

    async def initialize(self, timeout: int = 5, block: bool = False) -> bool:
        if self._addr is None:
            if not await self.find_device(timeout): 
                print("Failed to find the device")
                return False
        else:
            print(f"Using address {self._addr}")
            
        # Connect to the client
        self._client = BleakClient(self._addr, timeout = timeout)
        print("Attempting to connect to the client")
        try:
            await self._client.connect()
        except BleakDeviceNotFoundError:
            print("Unable to find device")
            return False
        
        if not await self.get_char_uuids():
            print("Failed to get the characteristic UUID's")
            return False
    
        print("Dispatching notification listener and writer")
        writer_task = asyncio.create_task(self.dispatch_writer())
        notification_task = asyncio.create_task(self.dispatch_notification_listener())

        if block:
            await asyncio.gather(notification_task, writer_task)
            
        print("Exiting initialization")
        return True
    
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
                        
        # Return if we initialized the UUID's for the Tx and Rx characteristics
        return self._tx_uuid is not None and self._rx_uuid is not None
    
    async def dispatch_notification_listener(self):
        try:
            # Start notification handler
            await self._client.start_notify(self._rx_uuid, self.notification_callback)
            
            # Keep the script running to receive notifications
            while True:
                # print("Yielding read task")
                await asyncio.sleep(1.0)
        
        except Exception as e:
            print(f"Error during notifications: {e}")
            raise
        finally:
            print("Stopping notifications")
            await self._client.stop_notify(self._rx_uuid)


    async def dispatch_writer(self):
        # cmd_motion = struct.pack('B', 0) + struct.pack('H', 12342) + struct.pack('H', 7891)
        # cmd_status = struct.pack('B', 1) + struct.pack('>I', (1 << 24) | 0x0)
        packet = None
        while True:
            # Wait for tx queue to get populated
            try:
                packet = self._tx_queue_state.get_nowait()
            except queue.Empty:
                pass
            
            if packet is None:
                # If there was no state command, get controller command
                try:
                    packet = self._tx_queue_cmd.get_nowait()
                except queue.Empty:
                    # yield
                    await asyncio.sleep(0)
                    continue
            
            # print(f"Transmitting packet: {packet}")
            
            # Transmit packet to the server
            await self._client.write_gatt_char(self._tx_uuid, packet)
            packet = None
            # print("Yielding write task")
            await asyncio.sleep(0) # yield task

def main():
    ble_listener = MotorboatBLEListener(
        device_name = "BlueNRG_Chat",
        service_desc = "Unknown",
        tx_property = "write",
        rx_property = "notify",
        address = "02:80:E1:00:00:AA"
    )
    
    asyncio.run(ble_listener.initialize(timeout = 30, block = True))
    
if __name__ == "__main__":
    main()
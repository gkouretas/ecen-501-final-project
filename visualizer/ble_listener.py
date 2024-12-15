import asyncio
from bleak import (
    BleakScanner, 
    BleakClient, 
    BleakGATTCharacteristic
)

class MotorboardBLEListener:
    def __init__(self, device_name: str, service_desc: str, tx_property: str, rx_property: str, address: str = None):
        self._name = device_name
        self._service_desc = service_desc
        self._tx_property = tx_property
        self._rx_property = rx_property
        
        self._addr: str = address
        
        self._tx_uuid: str
        self._rx_uuid: str
        
    async def initialize(self, timeout: int = 5) -> bool:
        if self._addr is None:
            if not await self.find_device(timeout): 
                print("Failed to find the device")
                return False
        else:
            print(f"Using address {self._addr}")
        
        if not await self.get_char_uuids():
            print("Failed to get the characteristic UUID's")
            return False
    
        print(f"{self._tx_uuid} {self._rx_uuid}")
    
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
        async with BleakClient(self._addr) as client:
            for service in client.services:
                if service.description == self._service_desc:
                    for char in service.characteristics:
                        for property in char.properties:
                            if property == self._tx_property:
                                self._tx_uuid = char
                            elif property == self._rx_property:
                                self._rx_uuid = char
                    
            print(f"{self._tx_uuid} {self._rx_uuid}")
            
            if self._rx_uuid is not None:
                data = await client.read_gatt_char(self._tx_uuid)
                print(f"Received data: {data}")
        # Return if we initialized the UUID's for the Tx and Rx characteristics
        return self._tx_uuid is not None and self._rx_uuid is not None

def main():
    ble_listener = MotorboardBLEListener(
        device_name = "BlueNRG_Chat",
        service_desc = "Unknown",
        tx_property = "notify",
        rx_property = "write",
        address = "02:80:E1:00:00:AA"
    )
    
    asyncio.run(ble_listener.initialize(timeout = 10))
    
if __name__ == "__main__":
    main()
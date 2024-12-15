import numpy as np
import asyncio
import sys

from boat_controller import BoatController
from motor_model import Motor
from boat_gui import BoatGUI
from ble_listener import MotorboatBLEListener
from qasync import QEventLoop

# Parameters used from https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling
_motor_params = {
    "R": 1.0,
    "L": 0.5,
    "J": 0.01,
    "b": 0.1,
    "K": 0.01
}

def main():    
    ble_comms = MotorboatBLEListener(
        device_name = "BlueNRG_Chat",
        service_desc = "Unknown",
        tx_property = "write",
        rx_property = "notify",
        address = "02:80:E1:00:00:AA"
    )
            
    controller = BoatController(
        motor_params = Motor.Params(**_motor_params),
        update_rate = 1.0 / 30.0, # Match sim rate to desired FPS
        ble_comms = ble_comms
    )
    
    b2gl_T = np.array([
        [ 0, 1, 0, 0],
        [-1, 0, 0, 0],
        [ 0, 0, 1, 0],
        [ 0, 0, 0, 1]
    ])
        
    gui = BoatGUI(controller, b2gl_T)
    
    loop = QEventLoop(gui.app)  # Create a QEventLoop for asyncio integration
    asyncio.set_event_loop(loop)  # Set it as the default event loop

    gui.run()

    with loop:  
        # Start the event loop
        loop.run_forever()

if __name__ == "__main__":
    main()
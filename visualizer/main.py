import numpy as np

from boat_controller import BoatController
from motor_model import Motor
from boat_gui import BoatGUI

# Parameters used from https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling
_motor_params = {
    "R": 1.0,
    "L": 0.5,
    "J": 0.01,
    "b": 0.1,
    "K": 0.01
}

def main():    
    controller = BoatController(
        motor_params = Motor.Params(**_motor_params),
        update_rate = 1.0 / 60.0 # Match sim rate to desired FPS
    )
    
    b2gl_T = np.array([
        [ 0, 1, 0, 0],
        [-1, 0, 0, 0],
        [ 0, 0, 1, 0],
        [ 0, 0, 0, 1]
    ])
    
    gui = BoatGUI(controller, b2gl_T)
    gui.run()

if __name__ == "__main__":
    main()
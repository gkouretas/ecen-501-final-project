import sys
import numpy as np

from boat_scene import BoatVisualizerScene
from boat_controller import BoatController
from motor_model import Motor
from joystick import JoystickWidget

# Parameters used from https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling
_motor_params = {
    "R": 1.0,
    "L": 0.5,
    "J": 0.01,
    "b": 0.1,
    "K": 0.01
}

def main():
    from PyQt5.QtWidgets import QApplication
    app = QApplication(sys.argv)
    
    controller = BoatController(
        motor_params = Motor.Params(**_motor_params),
        update_rate = 1 / 30.0,
        L = 60.0 # [mm], benchy length
    )
    
    b2gl_T = np.array([
        [ 0, 1, 0, 0],
        [-1, 0, 0, 0],
        [ 0, 0, 1, 0],
        [ 0, 0, 0, 1]
    ])
    
    view = BoatVisualizerScene(controller, b2gl_T)
    view.run()
    
    joystick = JoystickWidget(joystick_callback = controller.transmit_commands)
    joystick.show()
    
    app.exec_()

if __name__ == "__main__":
    main()
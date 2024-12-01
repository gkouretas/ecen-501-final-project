from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

from joystick_interface import JoystickWidget
from speed_interface import SpeedBar

from typing import Callable

class ControlInterface(QWidget):
    def __init__(self, control_callback: Callable[[float, float, int], None] = None):
        super().__init__()
        self.setMinimumSize(400, 400)


        
        self._joystick_widget = JoystickWidget()
        self._speed_bar = SpeedBar()
        
        self.setLayout(QGridLayout())
        self.layout().addWidget(self._joystick_widget, 0, 0)
        self.layout().addWidget(self._speed_bar, 0, 1)
        
        self._control_callback = control_callback
        
        if self._control_callback is not None:
            self._timer = QTimer()
            self._timer.timeout.connect(self._transmit_commands)
            self._timer.setInterval(33)
            self._timer.start()

    def _transmit_commands(self):
        self._control_callback(
            self._joystick_widget.x, 
            -self._joystick_widget.y, 
            self._speed_bar.value()
        )
        
    def keyPressEvent(self, a0):
        self._speed_bar.keyPressEvent(a0)
        return super().keyPressEvent(a0)
        
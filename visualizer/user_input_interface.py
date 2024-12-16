from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

from boat_controller import BoatController
from ble_listener import MotorState, BoatRequestedCommand

class ColoredCircle(QWidget):
    def __init__(self, c: Qt.GlobalColor, s: int):
        super().__init__()
        self._color = c
        self._size = s
        
        self.setMinimumSize(int(2.5*s), int(2.5*s))
    
    @property
    def color(self):
        return self._color
    
    @color.setter
    def color(self, c: Qt.GlobalColor):
        self._color = c
        
    def sizeHint(self):
        return QSize(self._size, self._size)

    def minimumSizeHint(self):
        return QSize(self._size, self._size)

    def update_view(self):
        return self.update()

    def paintEvent(self, a0):
        painter = QPainter(self)
        pen = QPen()
        painter.setBrush(self._color)
        painter.setPen(pen)
        painter.drawEllipse(
            QRectF(0, 0, 2*self._size, 2*self._size)
        )

class MotorStateIndicator(QWidget):
    def __init__(self, motor_index: int, c: Qt.GlobalColor, s: int):
        super().__init__()
        self.setLayout(QVBoxLayout())
        
        self._state_indicator = ColoredCircle(c, s)
        _label = QLabel(f"Motor #{motor_index}")
        
        self.layout().addWidget(_label, alignment = Qt.AlignmentFlag.AlignVCenter)
        self.layout().addWidget(self._state_indicator, alignment = Qt.AlignmentFlag.AlignVCenter)
        
        self.setMinimumHeight(s*3)
        
    def update_view(self):
        return self.update()
                
class UserInputWidget(QWidget):
    """
    User input widget, for monitoring and controlling the state of the motorboat
    """
    def __init__(self, num_motors: int, controller: BoatController) -> None:
        super().__init__()
        
        self.setLayout(QGridLayout())
        
        self._motor_states: list[MotorStateIndicator] = []
        for motor_index in range(num_motors):
            self._motor_states.append(
                MotorStateIndicator(motor_index, Qt.GlobalColor.gray, 25)
            )
            self.layout().addWidget(self._motor_states[-1], 0, motor_index, alignment = Qt.AlignmentFlag.AlignCenter)    
        
        self._controller = controller
        
        self._anchor_button = QPushButton("Anchor boat")
        self._anchor_button.pressed.connect(self._send_anchor_boat_command)
        self._remove_anchor = QPushButton("Lift anchor")
        self._remove_anchor.pressed.connect(self._remove_anchor_boat_command)
        self._recover_from_error = QPushButton("Recover from error")
        self._recover_from_error.pressed.connect(self._recover_from_error_boat_command)
        
        self.layout().addWidget(self._anchor_button, 1, 0, 1, 5)
        self.layout().addWidget(self._remove_anchor, 2, 0, 1, 5)
        self.layout().addWidget(self._recover_from_error, 3, 0, 1, 5)
        
        self._timer: QTimer = None
        
    def _send_anchor_boat_command(self):
        if not self._controller._ble_comms.send_boat_request(
            BoatRequestedCommand.ANCHOR_BOAT
        ):
            print("Failed to send command")

    def _remove_anchor_boat_command(self):
        if not self._controller._ble_comms.send_boat_request(
            BoatRequestedCommand.LIFT_ANCHOR
        ):
            print("Failed to send command")
            
    def _recover_from_error_boat_command(self):
        if not self._controller._ble_comms.send_boat_request(
            BoatRequestedCommand.RECOVERY_REQUEST
        ):
            print("Failed to send command")
        
    def update_view(self): 
        return self.update()
    
    def update_motor_state_color(self, motor_index: int, c: Qt.GlobalColor):
        self._motor_states[motor_index]._state_indicator.color = c
        
    def _state_to_color(self, state: MotorState):
        if state.is_active:
            if state.is_alive:
                if state.is_idle:
                    # Gray if alive but idle
                    return Qt.GlobalColor.gray
                else:
                    # Green if alive and moving
                    return Qt.GlobalColor.green
            else:
                # Red if inactive
                return Qt.GlobalColor.red
        else:
            # Black if spare/inactive
            return Qt.GlobalColor.black
        
    def refresh_boat_state(self):
        for motor_index, state in enumerate(self._controller._motor_states):
            self.update_motor_state_color(motor_index, self._state_to_color(state))
    
    def run(self):
        self._timer = QTimer()
        self._timer.timeout.connect(self.refresh_boat_state)
        self._timer.setInterval(50) # 20 Hz
        self._timer.start()
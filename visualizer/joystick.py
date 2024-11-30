import numpy as np
from functools import partial

from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from typing import Callable

MAX_JOYSTICK_VALUE = 1  

class JoystickWidget(QWidget):
    """
    Created based on https://stackoverflow.com/questions/55876713/how-to-create-a-joystick-controller-widget-in-pyqt
    """
    def __init__(self, joystick_callback: Callable[[float, float, int], None] = None, max_input: int = 100) -> None:
        super().__init__()
        self._joystick_position = QPointF(0.0, 0.0)
        self._joystick_callback = joystick_callback
        self._current_position: QRectF
        self._is_active: bool = False
        self._timer: QTimer = None
        self._input: int = 0
        self._max_input = max_input
        
        if joystick_callback is not None:
            self._timer = QTimer()
            self._timer.timeout.connect(self._transmit_commands)
            self._timer.setInterval(33)
            self._timer.start()
        
        self.update_joystick()
        
    def update_view(self): 
        return self.update()

    def update_joystick(self):
        self._current_position = self._get_circle_bounds(self.joystick_radius).translated(
            self.center + self._joystick_position
        )

    def mousePressEvent(self, evt):
        self._is_active = self._current_position.contains(evt.pos())
        return super().mousePressEvent(evt)
    
    def mouseReleaseEvent(self, _):
        self._is_active = False
        self._joystick_position = QPointF(0.0, 0.0) # return to (0, 0) when released
        self.update()
        
    def mouseMoveEvent(self, event):
        if self._is_active:
            p: QPoint = event.pos()
            if self.perimeter_radius >= self.euclidian_dist(p, self.center):
                self._joystick_position = p - self.center
            self.update()

    def paintEvent(self, _):
        # Draw bounding circle
        R = self.perimeter_radius
        self.update_joystick()
        
        painter = QPainter(self)
        pen = QPen()
        pen_mod = QPen()
        pen_mod.setWidth(5)

        painter.drawEllipse(self._get_circle_bounds(R).translated(self.center))

        # Draw joystick
        painter.setBrush(Qt.GlobalColor.gray)
        painter.drawEllipse(self._current_position)
        
        pen_mod.setColor(Qt.GlobalColor.gray)

        painter.setPen(pen)

    def _get_circle_bounds(self, R: float) -> QRectF:
        return QRectF(-R, -R, 2*R, 2*R)

    def _transmit_commands(self):
        self._joystick_callback(self.x, -self.y, self._input)

    def euclidian_dist(self, p1: QPoint, p2: QPoint):
        return ((p1.x() - p2.x())**2 + (p1.y() - p2.y()) ** 2) ** 0.5
    
    def keyPressEvent(self, evt):
        key = evt.key()
        if key == Qt.Key_Space:
            self._input = max(self._input + 1, self._max_input)
        return super().keyPressEvent(evt)
    
    def keyReleaseEvent(self, evt):
        if evt.isAutoRepeat(): return
        key = evt.key()
        if key == Qt.Key_Space:
            self._input = 0
        return super().keyReleaseEvent(evt)

    @property
    def perimeter_radius(self) -> float:
        return self.width() / 3
    
    @property
    def joystick_radius(self) -> float:
        return self.perimeter_radius / 4
    
    @property
    def center(self) -> QPointF:
        return QPointF(self.width() / 2.0, self.height() / 2.0)

    @property
    def normalized_position(self):
        return self._joystick_position / self.perimeter_radius
    
    @property
    def x(self):
        return self.normalized_position.x()
    
    @property
    def y(self):
        return self.normalized_position.y()

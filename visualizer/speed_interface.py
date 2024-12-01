from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

class SpeedBar(QProgressBar):
    def __init__(self):
        super().__init__()
        
        self.setOrientation(Qt.Orientation.Vertical)
        
        self.setMinimum(0)
        self.setMaximum(100)
        
        # Set initial speed value to zero
        self.setValue(0)
        
    def keyPressEvent(self, evt):
        key = evt.key()
        if key == Qt.Key.Key_Space:
            # Increase speed
            self.setValue(min(self.value() + 1, 100))
        elif key == Qt.Key.Key_B:
            # Decrease speed
            self.setValue(max(self.value() - 1, 0))
        return super().keyPressEvent(evt)
        
        
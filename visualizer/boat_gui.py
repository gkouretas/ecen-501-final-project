from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication

from boat_scene import BoatVisualizerScene
from boat_controller import BoatController
from control_interface import ControlInterface

from numpy.typing import NDArray

class BoatGUI(QMainWindow):
    def __init__(self, controller: BoatController, b2gl_T: NDArray):
        self.app = QApplication([])
        self.app.setApplicationName("Motorboat Simulation")
        
        super().__init__()
        
        self.setMinimumSize(800, 400)
        
        self._main_widget = QWidget()
        self._main_widget.setLayout(QGridLayout())
        self.setCentralWidget(self._main_widget)

        self._view = BoatVisualizerScene(controller, b2gl_T)        
        self._control = ControlInterface(control_callback = controller.transmit_commands)

        # Set to no focus so that key presses go through the main window
        self._view.setFocusPolicy(Qt.FocusPolicy.NoFocus)    
        self._control.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        self._main_widget.layout().addWidget(self._view, 0, 0)
        self._main_widget.layout().addWidget(self._control, 0, 1)
        
    def run(self):
        self._view.run()
        self.show()
        self.app.exec()
    
    def keyPressEvent(self, a0):
        self._view.keyPressEvent(a0)
        self._control.keyPressEvent(a0)
        return super().keyPressEvent(a0)
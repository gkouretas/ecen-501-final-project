import asyncio

from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication

from boat_scene import BoatVisualizerScene
from boat_controller import BoatController
from control_interface import ControlInterface
from ble_listener import MotorboatBLEListener

from functools import partial

from numpy.typing import NDArray

# Controls string, use HTML to preserve whitespace for easier centering of text
_control_string = \
"<style>p { line-height: 0.2; }</style>" + \
"<p><pre>Controls:                                                                                </pre></p>" + \
"<p><pre>[Left Click]   (Sim): Move camera, (Joystick): Command motorboat direction               </pre></p>" + \
"<p><pre>[Scroll]       (Sim): Zoom camera in/out                                                 </pre></p>" + \
"<p><pre>[Scroll Click] (Sim): Move camera                                                        </pre></p>" + \
"<p><pre>[SPACE]        Increase motorboat thrust                                                 </pre></p>" + \
"<p><pre>[B]            Decrease motorboat thrust                                                 </pre></p>" + \
"<p><pre>[H]            Move camera to home position (above boat, facing true north)              </pre></p>" + \
"<p><pre>[I]            Move camera to an isometric position                                      </pre></p>" + \
"<p><pre>[D]            Move camera to the side of the boat, provides a better view of water depth</pre></p>"

class BoatGUI(QMainWindow):
    def __init__(self, controller: BoatController, b2gl_T: NDArray):
        self.app = QApplication([])
        self.app.setApplicationName("Motorboat Simulation")
        
        super().__init__()
                
        self.setMinimumSize(1200, 700)
        
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
        
        _label = QLabel(_control_string)
        _label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        _label.setFont(QFont("Courier", 8, weight = QFont.Weight.Bold))
        
        self._main_widget.layout().addWidget(_label, 1, 0, 1, 2)
        self._ble_button = QPushButton("Connect to server")
        self._ble_button.clicked.connect(partial(self.connect_to_ble, controller._ble_comms))
        self._main_widget.layout().addWidget(self._ble_button, 2, 0, 1, 2)
        
    def connect_to_ble(self, comms: MotorboatBLEListener):
        self._ble_button.setEnabled(False)
        task = asyncio.create_task(comms.initialize(timeout = 30, block = False))
        task.add_done_callback(self._connection_complete)
        
    def _connection_complete(self, ret: bool):
        if not ret:
            print("BLE connection failed")
            self._ble_button.setEnabled(True)
        
    def run(self):
        self._view.run()
        self.showMaximized()
        # self.app.exec()
    
    def keyPressEvent(self, a0):
        self._view.keyPressEvent(a0)
        self._control.keyPressEvent(a0)
        if a0.key() == Qt.Key.Key_Escape:
            self.showNormal()
        return super().keyPressEvent(a0)
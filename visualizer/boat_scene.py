import numpy as np
from numpy.typing import NDArray

from stl import mesh
from pyqtgraph.opengl import (
    GLViewWidget, MeshData, GLMeshItem
)

from pyqtgraph.Transform3D import Transform3D

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.Qt3DCore import *
from PyQt5.QtWidgets import *
from PyQt5.Qt3DExtras import *

from stl import mesh

from boat_controller import BoatController

class BoatVisualizerScene(GLViewWidget):
    _MOTORBOAT_MESH_PATH = "visualizer/models/3DBenchy.stl"
    _WATER_RADIUS = 1000.0 # [mm]
    _BOAT_COLOR = (108/255, 64/255, 152/255, 1)
    _BACKGROUND_COLOR = (177, 181, 236)

    def __init__(self, controller: BoatController, boat_to_gl_transformation: NDArray):
        super().__init__()
        
        self._controller = controller
        self._counter = 0
        
        self._boat_mesh_item = self._init_boat_mesh()
        self._water_mesh_item = self._init_water_mesh()
        self._timer: QTimer = None
                
        self._boat_mesh_item.setTransform(Transform3D(*(np.linalg.inv(boat_to_gl_transformation).flatten())))
        self.setBackgroundColor(self._BACKGROUND_COLOR)
        
        self.addItem(self._boat_mesh_item)
        self.addItem(self._water_mesh_item)
        
    def _init_boat_mesh(self) -> GLMeshItem:
        _mesh = mesh.Mesh.from_file(self._MOTORBOAT_MESH_PATH)
        _pts = np.asarray(_mesh.points).reshape(-1, 3)
        _faces = np.arange(_pts.shape[0]).reshape(-1, 3)
                
        return GLMeshItem(
            meshdata = MeshData(vertexes = _pts, faces = _faces), 
            smooth = True, 
            drawFaces = True, 
            drawEdges = False, 
            color = self._BOAT_COLOR
        )
        
    def _init_water_mesh(self) -> GLMeshItem:
        _meshdata = MeshData.cylinder(
            rows = 25, 
            cols = 25, 
            radius = [+self._WATER_RADIUS, +self._WATER_RADIUS], 
            length = -self._controller.depth,
        )
        
        _circle = np.array([np.cos(np.linspace(0, 2*np.pi, num = 25)), np.sin(np.linspace(0, 2*np.pi, num = 25)), np.zeros(25)]).T
                
        _meshdata.setVertexes(
            np.concatenate((_circle, _meshdata.vertexes()))
        )
        
        _mesh = GLMeshItem(
            meshdata = _meshdata,
            smooth = False,
            drawFaces = True,
            drawEdges = False,
            color = (0, 0, 1, 0.5)
        )
        
        # Uncomment for water to be transparent        
        # _mesh.setGLOptions('additive')

        return _mesh
    
    def paintEvent(self, event):
        super().paintEvent(event)

        # Create a QPainter to overlay text
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Set font and color
        painter.setPen(Qt.black)
        painter.setFont(self.font())
        
        # Draw text
        painter.drawText(15, 15, f"Orientation: {self._controller.heading:.3f} deg")
        painter.drawText(15, 25, f"X: {self._controller.x:.3f} mm")
        painter.drawText(15, 35, f"Y: {self._controller.y:.3f} mm")
        
        # End painter
        painter.end()

    def run(self):
        self._timer = QTimer()
        self._timer.timeout.connect(self.refresh)
        self._timer.setInterval(int(self._controller.update_rate))
        self._timer.start()
        
        self.show()
        
    def update_view(self):
        self.refresh()
    
    def refresh(self):
        # TODO: get BLE packet and set boat translation + water depth accordingly...
        self._controller.run_kinematics()
        self._boat_mesh_item.applyTransform(Transform3D(*self._controller.dT.flatten()), local = False)
        
        # self._boat_mesh_item.rotate(self._controller.dh, 0, 0, 1, local = True)
        # self._boat_mesh_item.translate(self._controller.dy, -self._controller.dx, 0, local = False)
        # self._boat_mesh_item.translate(self._controller.dx, 0, 0, local = False)
        # print(gl_trans_rel.flatten())
        
        self.update()
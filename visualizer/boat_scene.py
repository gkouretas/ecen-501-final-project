import sys
import numpy as np

from stl import mesh
from pyqtgraph.opengl import (
    GLViewWidget, MeshData, GLMeshItem
)

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.Qt3DCore import *
from PyQt5.QtWidgets import *
from PyQt5.Qt3DExtras import *

from stl import mesh

class BoatVisualizerScene(GLViewWidget):
    _MOTORBOAT_MESH_PATH = "visualizer/models/3DBenchy.stl"
    _WATER_RADIUS = 1000.0
    _BACKGROUND = (0, 0, 0, 1)

    def __init__(self):
        super().__init__()
        
        self._depth = 10.0
        self._counter = 0
        
        self._boat_mesh_item = self._init_boat_mesh()
        self._water_mesh_item = self._init_water_mesh()
        
        self.setBackgroundColor(self._BACKGROUND)
        
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
            drawEdges = True, 
            color = (1, 0, 0, 1),
            edgeColor = (1, 0, 0, 1)
        )
        
    def _init_water_mesh(self) -> GLMeshItem:
        _mesh = GLMeshItem(
            meshdata = MeshData.cylinder(
                rows = 25, 
                cols = 25, 
                radius = [+self._WATER_RADIUS, +self._WATER_RADIUS], 
                length = -self._depth,
            ),
            smooth = False,
            drawFaces = True,
            drawEdges = False,
            color = (0, 0, 1, 0.5)
        )
        
        # TODO: add surface for water in addition to cylinder column
        
        _mesh.setGLOptions('additive')

        return _mesh
    
    def run(self):
        timer = QTimer()
        timer.timeout.connect(self.refresh)
        timer.setInterval(33)
        timer.start()
        
        self.show()
    
    def refresh(self):
        # TODO: get BLE packet and set boat translation + water depth accordingly...
        self.update()

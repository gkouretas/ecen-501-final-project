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
    _BOAT_COLOR = (0/255, 80/255, 0/255, 1)

    _BACKGROUND_COLOR = (177, 181, 236)

    def __init__(self, controller: BoatController, boat_to_gl_transformation: NDArray):
        super().__init__()
        
        self._controller = controller
        
        self._boat_mesh_item = self._init_boat_mesh()
        self._water_mesh_item = self._init_water_mesh()
        self._timer: QTimer = None
        
        self._home_tform = Transform3D(*(np.linalg.inv(boat_to_gl_transformation).flatten()))
                
        self._boat_mesh_item.setTransform(self._home_tform)
        self.setBackgroundColor(self._BACKGROUND_COLOR)
        self.setCameraParams(**self.home_camera_params())
        
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
        _meshdata = self._get_water_mesh()
        
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
    
    def _get_water_mesh(self):
        _meshdata = MeshData.cylinder(
            rows = 25, 
            cols = 25, 
            radius = [+self._WATER_RADIUS, +self._WATER_RADIUS], 
            length = -self._controller.depth,
        )
        
        _circle = np.array(
            [np.cos(np.linspace(0, 2*np.pi, num = 25)), np.sin(np.linspace(0, 2*np.pi, num = 25)), np.zeros(25)]
        ).T
                
        _meshdata.setVertexes(
            np.concatenate((_circle, _meshdata.vertexes()))
        )
        
        return _meshdata
    
    def home_camera_params(self):
        return {
            "center": QVector3D(self._controller.x, self._controller.y, 0.0),
            "distance": 250.0,
            "elevation": 90.0,
            "azimuth": -90.0
        }

    def isometric_camera_params(self):
        return {
            "center": QVector3D(self._controller.x, self._controller.y, 0.0),
            "distance": 250.0,
            "elevation": 5.0,
            "azimuth": 135.0 + self._controller.heading
        }
        
    def depth_camera_params(self):
        return {
            "center": QVector3D(self._controller.x, self._controller.y, 0.0),
            "distance": self._WATER_RADIUS,
            "elevation": 0.0,
            "azimuth": 180.0 + self._controller.heading
        }
    
    def paintEvent(self, event):
        super().paintEvent(event)

        # Create a QPainter to overlay text
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Set font and color
        painter.setPen(Qt.GlobalColor.white)
        painter.setFont(QFont("Courier", 8, weight = QFont.Weight.Bold))
        
        # Draw text
        painter.drawText(15, 15, f"STM32 Timestamp: {self._controller.timestamp} ms")
        painter.drawText(15, 25, f"State: {self._controller.state.name}")
        painter.drawText(15, 35, f"X: {self._controller.x:.3f} mm")
        painter.drawText(15, 45, f"Y: {self._controller.y:.3f} mm")
        painter.drawText(15, 55, f"Delta: {self._controller.delta:.3f} deg")
        painter.drawText(15, 65, f"Velocity: {self._controller.velocity:.3f} mm/s")
        painter.drawText(15, 75, f"Heading: {self._controller.heading:.3f} deg")
        painter.drawText(15, 85, f"Roll: {self._controller.roll:.3f} deg")
        painter.drawText(15, 95, f"Pitch: {self._controller.pitch:.3f} deg")
        painter.drawText(15, 105, f"Depth: {self._controller.depth} mm")
        painter.drawText(15, 115, f"Temperature: {self._controller.temperature} °C")
                
        # End painter
        painter.end()

    def run(self):
        self._timer = QTimer()
        self._timer.timeout.connect(self.refresh)
        self._timer.setInterval(int(self._controller.update_rate * 1000))
        self._timer.start()
                
    def update_view(self):
        self.refresh()
    
    def refresh(self):
        self._controller.run_kinematics()
        self._boat_mesh_item.setTransform(self._home_tform)
        self._boat_mesh_item.translate(dx = self._controller.x, dy = self._controller.y, dz = 0.0, local = False)
        self._boat_mesh_item.rotate(angle = self._controller.heading, x = 0, y = 0, z = 1, local = True)
        self._boat_mesh_item.rotate(angle = self._controller.roll, x = 1, y = 0, z = 0, local = True)
        self._boat_mesh_item.rotate(angle = self._controller.pitch, x = 0, y = 1, z = 0, local = True)
             
        # Update visual of water depth      
        self._water_mesh_item.setMeshData(meshdata = self._get_water_mesh())
                        
        self.update()
        
    def keyPressEvent(self, ev):
        key = ev.key()
        if key == Qt.Key.Key_H:
            self.setCameraParams(**self.home_camera_params())
        elif key == Qt.Key.Key_I:
            self.setCameraParams(**self.isometric_camera_params())
        elif key == Qt.Key.Key_D:
            self.setCameraParams(**self.depth_camera_params())
            
        return super().keyPressEvent(ev)
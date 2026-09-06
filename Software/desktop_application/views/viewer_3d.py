import math
import numpy as np
from PyQt6.QtWidgets import QVBoxLayout
from PyQt6.QtGui import QVector3D, QMatrix4x4
import pyqtgraph.opengl as gl

class Drone3DViewer:

    def __init__(self, container_widget):
        self.container = container_widget
        self.setupUi()

    def setupUi(self):
        """Configures the PyOpenGL widget inside the Designer container."""
        if self.container.layout() is None:
            self.container.setLayout(QVBoxLayout())

        self.view_3d = gl.GLViewWidget()
        self.view_3d.setCameraPosition(distance=20, elevation=30, azimuth=45)
        self.container.layout().addWidget(self.view_3d)

        self.grid = gl.GLGridItem()
        self.grid.scale(2, 2, 2)
        self.view_3d.addItem(self.grid)

        self.trail_history_len = 150
        self.trail_data = np.zeros((self.trail_history_len, 3))
        self.trail_item = gl.GLLinePlotItem(pos=self.trail_data, color=(1, 0, 1, 0.8), width=3, antialias=True)
        self.view_3d.addItem(self.trail_item)

        vertices = np.array([
            [-1.0, -1.0, 0.0], [1.0,  1.0, 0.0],
            [-1.0,  1.0, 0.0], [1.0, -1.0, 0.0],
            [ 0.0,  0.0, 0.0], [0.8,  0.0, 0.3]
        ])
        
        colors = np.array([
            [0.0, 1.0, 1.0, 1.0], [0.0, 1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0, 1.0], [0.0, 1.0, 1.0, 1.0],
            [1.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 1.0]
        ])

        self.drone_model = gl.GLLinePlotItem(pos=vertices, color=colors, mode='lines', width=4, antialias=True)
        self.view_3d.addItem(self.drone_model)

    def updateModel(self, data):
        """Transforms radians to degrees and redraws the 3D model."""
        roll_deg  = data.get('R', 0.0) * (180.0 / math.pi)
        pitch_deg = data.get('P', 0.0) * (180.0 / math.pi)
        pos_x     = data.get('X', 0.0)
        pos_y     = data.get('Y', 0.0)
        alt_z     = data.get('Z', 0.0)

        drone_matrix = QMatrix4x4()
        drone_matrix.translate(pos_x, pos_y, alt_z)
        drone_matrix.rotate(roll_deg, 1, 0, 0)
        drone_matrix.rotate(pitch_deg, 0, 1, 0)
        self.drone_model.setTransform(drone_matrix)

        grid_matrix = QMatrix4x4()
        grid_matrix.translate(pos_x, pos_y, 0.0)
        self.grid.setTransform(grid_matrix)

        new_point = np.array([[pos_x, pos_y, alt_z]])
        self.trail_data = np.vstack((self.trail_data[1:], new_point))
        self.trail_item.setData(pos=self.trail_data)

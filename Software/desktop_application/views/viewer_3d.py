import math
from PyQt6.QtWidgets import QVBoxLayout
from PyQt6.QtGui import QVector3D
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
        self.view_3d.setCameraPosition(distance=10, elevation=30, azimuth=45)
        self.container.layout().addWidget(self.view_3d)

        grid = gl.GLGridItem()
        grid.scale(2, 2, 2)
        self.view_3d.addItem(grid)

        self.drone_model = gl.GLAxisItem(size=QVector3D(2, 2, 2))
        self.view_3d.addItem(self.drone_model)

    def updateModel(self, data):
        """Transforms radians to degrees and redraws the 3D model."""
        roll_deg  = data.get('R', 0.0) * (180.0 / math.pi)
        pitch_deg = data.get('P', 0.0) * (180.0 / math.pi)
        alt_z     = data.get('Z', 0.0)

        self.drone_model.resetTransform()
        self.drone_model.translate(0, 0, alt_z)
        self.drone_model.rotate(roll_deg, 1, 0, 0)
        self.drone_model.rotate(pitch_deg, 0, 1, 0)

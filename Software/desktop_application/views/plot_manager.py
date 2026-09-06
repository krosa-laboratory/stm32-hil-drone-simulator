import collections
from PyQt6.QtWidgets import QVBoxLayout
import pyqtgraph as pg

class PlotManager:

    def __init__(self, container_widget):
        self.container = container_widget
        self.setupUi()

    def setupUi(self):
        """Configures the pyqtgraph widgets inside the provided container."""
        if self.container.layout() is None:
            self.container.setLayout(QVBoxLayout())

        pg.setConfigOption('background', 'k')
        pg.setConfigOption('foreground', 'w')

        self.history_len = 100
        self.data_history = {
            'R':     collections.deque(maxlen=self.history_len),
            'R_ref': collections.deque(maxlen=self.history_len),
            'P':     collections.deque(maxlen=self.history_len),
            'P_ref': collections.deque(maxlen=self.history_len),
            'Z':     collections.deque(maxlen=self.history_len),
            'X':     collections.deque(maxlen=self.history_len),
            'X_ref': collections.deque(maxlen=self.history_len),
            'Y':     collections.deque(maxlen=self.history_len),
            'Y_ref': collections.deque(maxlen=self.history_len)
        }

        # Plot 1: Roll
        self.plot_roll = pg.PlotWidget(title="Roll (Degrees)")
        self.plot_roll.showGrid(x=True, y=True)
        self.curve_roll = self.plot_roll.plot(pen=pg.mkPen('r', width=2), name="Actual")
        self.curve_roll_ref = self.plot_roll.plot(pen=pg.mkPen('y', width=2, style=pg.QtCore.Qt.PenStyle.DashLine), name="Setpoint")
        self.container.layout().addWidget(self.plot_roll)

        # Plot 2: Pitch
        self.plot_pitch = pg.PlotWidget(title="Pitch (Degrees)")
        self.plot_pitch.showGrid(x=True, y=True)
        self.curve_pitch = self.plot_pitch.plot(pen=pg.mkPen('g', width=2), name="Actual")
        self.curve_pitch_ref = self.plot_pitch.plot(pen=pg.mkPen('y', width=2, style=pg.QtCore.Qt.PenStyle.DashLine))
        self.container.layout().addWidget(self.plot_pitch)

        # Plot 3: Z Altitude
        self.plot_z = pg.PlotWidget(title="Z Altitude (Meters)")
        self.plot_z.showGrid(x=True, y=True)
        self.curve_z = self.plot_z.plot(pen=pg.mkPen('c', width=2))
        self.container.layout().addWidget(self.plot_z)

        # Plot 4: X-Y plane view
        self.plot_xy = pg.PlotWidget(title="Trajectory X-Y (Top-Down View)")
        self.plot_xy.showGrid(x=True, y=True)
        self.plot_xy.setLabel('bottom', 'X Position (m)')
        self.plot_xy.setLabel('left', 'Y Position (m)')
        self.plot_xy.setAspectLocked(True)
        self.plot_xy.setXRange(-5, 5)
        self.plot_xy.setYRange(-5, 5)
        self.curve_xy_trail = self.plot_xy.plot(pen=pg.mkPen('#ff00ff', width=1.5, style=pg.QtCore.Qt.PenStyle.DotLine))
        self.scatter_current = pg.ScatterPlotItem(size=12, brush=pg.mkBrush('#00ffcc'), symbol='o')
        self.plot_xy.addItem(self.scatter_current)
        self.scatter_ref = pg.ScatterPlotItem(size=10, pen=pg.mkPen('#ffff00', width=2), symbol='x')
        self.plot_xy.addItem(self.scatter_ref)

        self.container.layout().addWidget(self.plot_xy)

    def updateData(self, data):
        """Adds new data to the history and redraws the curves."""
        self.data_history['R'].append(data.get('R', 0.0))
        self.data_history['R_ref'].append(data.get('R_ref', 0.0))
        self.data_history['P'].append(data.get('P', 0.0))
        self.data_history['P_ref'].append(data.get('P_ref', 0.0))
        self.data_history['Z'].append(data.get('Z', 0.0))
        self.data_history['X'].append(data.get('X', 0.0))
        self.data_history['X_ref'].append(data.get('X_ref', 0.0))
        self.data_history['Y'].append(data.get('Y', 0.0))
        self.data_history['Y_ref'].append(data.get('Y_ref', 0.0))

        self.curve_roll.setData(list(self.data_history['R']))
        self.curve_roll_ref.setData(list(self.data_history['R_ref']))

        self.curve_pitch.setData(list(self.data_history['P']))
        self.curve_pitch_ref.setData(list(self.data_history['P_ref']))

        self.curve_z.setData(list(self.data_history['Z']))

        x_list = list(self.data_history['X'])
        y_list = list(self.data_history['Y'])
        x_ref_list = list(self.data_history['X_ref'])
        y_ref_list = list(self.data_history['Y_ref'])

        self.curve_xy_trail.setData(x_list, y_list)

        if x_list and y_list:
            current_x = x_list[-1]
            current_y = y_list[-1]

            self.scatter_current.setData([current_x], [current_y])

            span = 5.0 
            self.plot_xy.setXRange(current_x - span, current_x + span)
            self.plot_xy.setYRange(current_y - span, current_y + span)

        if x_ref_list and y_ref_list:
            self.scatter_ref.setData([x_ref_list[-1]], [y_ref_list[-1]])

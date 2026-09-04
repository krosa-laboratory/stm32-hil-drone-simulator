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

    def updateData(self, data):
        """Adds new data to the history and redraws the curves."""
        self.data_history['R'].append(data.get('R', 0.0))
        self.data_history['R_ref'].append(data.get('R_ref', 0.0))
        self.data_history['P'].append(data.get('P', 0.0))
        self.data_history['P_ref'].append(data.get('P_ref', 0.0))
        self.data_history['Z'].append(data.get('Z', 0.0))

        self.curve_roll.setData(list(self.data_history['R']))
        self.curve_roll_ref.setData(list(self.data_history['R_ref']))

        self.curve_pitch.setData(list(self.data_history['P']))
        self.curve_pitch_ref.setData(list(self.data_history['P_ref']))

        self.curve_z.setData(list(self.data_history['Z']))

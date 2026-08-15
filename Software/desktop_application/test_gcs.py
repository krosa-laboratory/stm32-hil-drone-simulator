import sys
import os
import collections
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout
from PyQt6 import uic
import pyqtgraph as pg
from telemetry_core import TelemetryEngine

class AdvancedGCS(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Load the Qt Designer layout
        ui_file = os.path.join(os.path.dirname(__file__), 'gcs_layout.ui')
        uic.loadUi(ui_file, self)

        self.setWindowTitle("STM32 HIL Drone Control Station")

        # INITIALIZE PLOTS
        self.init_plots()

        # INITIALIZE TELEMETRY ENGINE
        self.telemetry = TelemetryEngine(port_name='COM3') 
        
        # CONNECT SIGNALS
        self.telemetry.connection_status.connect(self.update_status)
        self.telemetry.data_updated.connect(self.update_display)
        self.telemetry.data_updated.connect(self.update_plots) # Connect data to the plots

        # CAPTURE KEYBOARD EVENTS
        self.keyPressEvent = self.handle_keypress 

        # Start hardware
        self.telemetry.start()

    def init_plots(self):
        """Configures the pyqtgraph widgets inside the Designer container."""
        # Ensure the designer container has a layout
        if self.container_plots.layout() is None:
            self.container_plots.setLayout(QVBoxLayout())

        # Global plot style (Black background, white text)
        pg.setConfigOption('background', 'k')
        pg.setConfigOption('foreground', 'w')

        # Variables to store data history (Last 100 points)
        self.history_len = 100
        self.data_history = {
            'R': collections.deque(maxlen=self.history_len),
            'R_ref': collections.deque(maxlen=self.history_len),
            'P': collections.deque(maxlen=self.history_len),
            'P_ref': collections.deque(maxlen=self.history_len),
            'Z': collections.deque(maxlen=self.history_len),
            'Z_ref': collections.deque(maxlen=self.history_len) # Assuming Z_ref is added to telemetry later
        }

        # Plot 1: Roll (Red)
        self.plot_roll = pg.PlotWidget(title="Roll (Degrees)")
        self.plot_roll.showGrid(x=True, y=True)
        self.curve_roll = self.plot_roll.plot(pen=pg.mkPen('r', width=2), name="Actual")
        self.curve_roll_ref = self.plot_roll.plot(pen=pg.mkPen('y', width=2, style=pg.QtCore.Qt.PenStyle.DashLine), name="Setpoint")
        self.container_plots.layout().addWidget(self.plot_roll)

        # Plot 2: Pitch (Green)
        self.plot_pitch = pg.PlotWidget(title="Pitch (Degrees)")
        self.plot_pitch.showGrid(x=True, y=True)
        self.curve_pitch = self.plot_pitch.plot(pen=pg.mkPen('g', width=2))
        self.curve_pitch_ref = self.plot_pitch.plot(pen=pg.mkPen('y', width=2, style=pg.QtCore.Qt.PenStyle.DashLine))
        self.container_plots.layout().addWidget(self.plot_pitch)

        # Plot 3: Z Altitude (Blue)
        self.plot_z = pg.PlotWidget(title="Z Altitude (Meters)")
        self.plot_z.showGrid(x=True, y=True)
        self.curve_z = self.plot_z.plot(pen=pg.mkPen('c', width=2))
        # self.curve_z_ref = self.plot_z.plot(pen=pg.mkPen('y', width=2, style=pg.QtCore.Qt.PenStyle.DashLine)) # Optional
        self.container_plots.layout().addWidget(self.plot_z)

    def update_plots(self, data):
        """Adds new data to the history and redraws the curves."""
        # Fill with zeros if any data is missing initially to avoid errors
        roll = data.get('R', 0.0)
        roll_ref = data.get('R_ref', 0.0)
        pitch = data.get('P', 0.0)
        pitch_ref = data.get('P_ref', 0.0)
        z = data.get('Z', 0.0)
        
        # Add to history
        self.data_history['R'].append(roll)
        self.data_history['R_ref'].append(roll_ref)
        self.data_history['P'].append(pitch)
        self.data_history['P_ref'].append(pitch_ref)
        self.data_history['Z'].append(z)

        # Update the lines in the plots
        self.curve_roll.setData(list(self.data_history['R']))
        self.curve_roll_ref.setData(list(self.data_history['R_ref']))
        
        self.curve_pitch.setData(list(self.data_history['P']))
        self.curve_pitch_ref.setData(list(self.data_history['P_ref']))

        self.curve_z.setData(list(self.data_history['Z']))

    def update_status(self, is_connected, msg):
        self.statusBar().showMessage(f"Data Link: {msg}")

    def update_display(self, data):
        display_str = (
            f"🚀 ACTUAL ATTITUDE:\n"
            f"Altitude (Z): {data.get('Z', 0.0):.2f} m\n"
            f"Roll (R): {data.get('R', 0.0):.2f}° | Pitch (P): {data.get('P', 0.0):.2f}°\n"
            f"---------------------------\n"
            f"🎮 SETPOINTS (Desired):\n"
            f"Roll_Ref: {data.get('R_ref', 0.0):.2f}° | Pitch_Ref: {data.get('P_ref', 0.0):.2f}°\n"
            f"Throttle: {data.get('U1', 0.0):.2f} N"
        )
        self.label_telemetry.setText(display_str)

    def handle_keypress(self, event):
        key = event.text().upper()
        valid_keys = ['W', 'A', 'S', 'D', 'R', 'F']
        if key in valid_keys:
            self.telemetry.send_command(key)

    def closeEvent(self, event):
        self.telemetry.stop()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = AdvancedGCS()
    window.show()
    sys.exit(app.exec())

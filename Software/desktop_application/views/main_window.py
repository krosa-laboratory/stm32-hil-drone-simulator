import os
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QCloseEvent
from PyQt6.QtWidgets import QMainWindow, QWidget
from PyQt6 import uic
from telemetry_core import TelemetryEngine
from views.plot_manager import PlotManager
from views.viewer_3d import Drone3DViewer

class AdvancedGCS(QMainWindow):

    def __init__(self):
        super().__init__()

        # Load the Qt Designer layout
        ui_file = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'gcs_layout.ui')
        uic.loadUi(ui_file, self)
        self.setWindowTitle("STM32G4 HIL Drone Control Station")

        # Instantiate modular views
        self.plot_manager = PlotManager(self.container_plots)
        self.viewer_3d = Drone3DViewer(self.container_3d)

        # Initialize telemetry engine
        self.telemetry = TelemetryEngine(port_name='COM3')

        # Route signals to the specific modules
        self.telemetry.connection_status.connect(self.updateStatus)
        self.telemetry.data_updated.connect(self.updateDisplay)
        self.telemetry.data_updated.connect(self.plot_manager.updateData)
        self.telemetry.data_updated.connect(self.viewer_3d.updateModel)
        self.telemetry.pid_sync_received.connect(self.sync_gui_pids)
        self.btn_send_pid.clicked.connect(self.send_pid_gains)

        self.keyPressEvent = self.handleKeypress
        self.telemetry.start()
        self.setFocus()

    def updateStatus(self, is_connected, msg):
        self.statusBar().showMessage(f"Data Link: {msg}")

    def updateDisplay(self, data):
        display_str = (
            f"ACTUAL ATTITUDE:\n"
            f"Altitude (Z): {data.get('Z', 0.0):.2f} m\n"
            f"Roll (R): {data.get('R', 0.0):.2f}° | Pitch (P): {data.get('P', 0.0):.2f}°\n\n"
            f"---------------------------\n\n"
            f"SETPOINTS (Desired):\n"
            f"Roll_Ref: {data.get('R_ref', 0.0):.2f}° | Pitch_Ref: {data.get('P_ref', 0.0):.2f}°\n"
            f"Throttle: {data.get('U1', 0.0):.2f} N"
        )
        self.label_telemetry.setText(display_str)

    def sync_gui_pids(self, kp, ki, kd):
        """Updates the UI spinboxes with the actual values retrieved from the STM32."""
        self.spin_kp.setValue(kp)
        self.spin_ki.setValue(ki)
        self.spin_kd.setValue(kd)

        self.statusBar().showMessage(f"PID State Synced from MCU -> Kp: {kp}, Ki: {ki}, Kd: {kd}", 5000)

    def send_pid_gains(self):
        """Serialize PID inputs and routes them to the STM32 via USB."""
        kp = self.spin_kp.value()
        ki = self.spin_ki.value()
        kd = self.spin_kd.value()

        # Payload format required by the firmware
        command_frame = f"P:{kp:.3f},I:{ki:.3f},D:{kd:.3f}\n"

        # Inject directly through the hardware serial port
        if self.telemetry and self.telemetry.is_running:
            try:
                self.telemetry.serial_port.write(command_frame.encode('utf-8'))
                self.statusBar().showMessage(f"PIDs Sent: {command_frame.strip()}")
            except Exception as e:
                self.statusBar().showMessage(f"TX Error: {str(e)}")

    def handleKeypress(self, event):
        key = event.text().upper()
        if key in ['W', 'A', 'S', 'D', 'R', 'F']:
            self.telemetry.send_command(key)

    def closeEvent(self, event):
        self.telemetry.stop()
        event.accept()

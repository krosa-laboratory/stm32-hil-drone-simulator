import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QPushButton
from telemetry_core import TelemetryEngine

class SimpleGCS(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone GCS - Telemetry Test")
        self.resize(300, 200)

        # Interfaz básica
        self.label_status = QLabel("Desconectado")
        self.label_data = QLabel("Esperando telemetría...")
        self.btn_up = QPushButton("Subir Z (Tecla R)")
        
        layout = QVBoxLayout()
        layout.addWidget(self.label_status)
        layout.addWidget(self.label_data)
        layout.addWidget(self.btn_up)

        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

        # --- INICIALIZAR EL MOTOR DE TELEMETRÍA ---
        # ATENCIÓN: Cambia 'COM3' (Windows) o '/dev/ttyACM0' (Linux/Mac) por el de tu placa
        self.telemetry = TelemetryEngine(port_name='COM3') 

        # Conectar las señales del hilo a las funciones de nuestra GUI
        self.telemetry.connection_status.connect(self.update_status)
        self.telemetry.data_updated.connect(self.update_display)
        
        # Conectar el botón al envío de comandos
        self.btn_up.clicked.connect(lambda: self.telemetry.send_command('R'))

        # Arrancar el hilo secundario
        self.telemetry.start()

    def update_status(self, is_connected, msg):
        self.label_status.setText(f"Estado: {msg}")

    def update_display(self, data):
        # Esta función se llama mágicamente cada vez que llega un dato
        display_str = (
            f"Altitud (Z): {data.get('Z', 0.0):.2f} m\n"
            f"Roll (R): {data.get('R', 0.0):.2f}°\n"
            f"Pitch (P): {data.get('P', 0.0):.2f}°\n"
            f"Throttle (U1): {data.get('U1', 0.0):.2f} N"
        )
        self.label_data.setText(display_str)

    def closeEvent(self, event): # type: ignore
        # Apagar el hilo de forma limpia al cerrar la ventana
        self.telemetry.stop()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SimpleGCS()
    window.show()
    sys.exit(app.exec())

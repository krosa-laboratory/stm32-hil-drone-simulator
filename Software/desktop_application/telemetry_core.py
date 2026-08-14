import serial
import time
from PyQt6.QtCore import QThread, pyqtSignal

class TelemetryEngine(QThread):
    # Definimos las señales que este hilo emitirá hacia la GUI
    # data_updated enviará un diccionario con { 'R': 0.0, 'Z': 5.0, ... }
    data_updated = pyqtSignal(dict)
    connection_status = pyqtSignal(bool, str)

    def __init__(self, port_name, baudrate=115200):
        super().__init__()
        self.port_name = port_name
        self.baudrate = baudrate
        self.serial_port = None
        self.is_running = False

    def run(self):
        """ Este es el bucle que corre en segundo plano (Worker Thread) """
        self.is_running = True
        try:
            # En USB CDC virtual, el baudrate real lo dicta el hardware, 
            # pero pyserial requiere el parámetro.
            self.serial_port = serial.Serial(self.port_name, self.baudrate, timeout=1)
            self.connection_status.emit(True, f"Conectado a {self.port_name}")
            
            # Limpiamos el buffer de entrada por si había basura acumulada
            self.serial_port.reset_input_buffer()

            while self.is_running:
                if self.serial_port.in_waiting > 0:
                    # Leemos la línea hasta el \r\n
                    raw_data = self.serial_port.readline()
                    
                    try:
                        # Decodificamos bytes a string UTF-8
                        text_data = raw_data.decode('utf-8').strip()
                        
                        # Parseamos el CSV: "R:0.00,R_ref:0.00,Z:5.00,U1:12.3"
                        # Lo convertimos a un diccionario de Python de forma elegante
                        parsed_dict = {}
                        pairs = text_data.split(',')
                        for pair in pairs:
                            if ':' in pair:
                                key, value = pair.split(':')
                                parsed_dict[key.strip()] = float(value.strip())
                        
                        # Emitimos el diccionario hacia la GUI (Thread-safe)
                        if parsed_dict:
                            self.data_updated.emit(parsed_dict)
                            
                    except (UnicodeDecodeError, ValueError) as e:
                        # Si llega un byte corrupto, lo ignoramos y seguimos
                        # En sistemas críticos no queremos que la app crashee por un bit volteado
                        pass 

        except serial.SerialException as e:
            self.connection_status.emit(False, f"Error de puerto: {str(e)}")
        finally:
            self.close_port()

    def send_command(self, cmd_char):
        """ Permite inyectar comandos desde la GUI hacia el STM32 """
        if self.serial_port and self.serial_port.is_open:
            try:
                # Enviamos el carácter (ej. 'W') codificado en bytes
                self.serial_port.write(cmd_char.encode('utf-8'))
            except serial.SerialException:
                pass

    def close_port(self):
        self.is_running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.connection_status.emit(False, "Desconectado")

    def stop(self):
        self.is_running = False
        self.wait() # Esperamos a que el hilo termine de forma segura

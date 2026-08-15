import serial
import time
from PyQt6.QtCore import QThread, pyqtSignal

class TelemetryEngine(QThread):
    # Define the signals that this thread will emit to the GUI
    # data_updated will send a dictionary like { 'R': 0.0, 'Z': 5.0, ... }
    data_updated = pyqtSignal(dict)
    connection_status = pyqtSignal(bool, str)

    def __init__(self, port_name, baudrate=115200):
        super().__init__()
        self.port_name = port_name
        self.baudrate = baudrate
        self.serial_port = None
        self.is_running = False

    def run(self):
        """ This is the background worker thread loop """
        self.is_running = True
        try:
            # In virtual USB CDC, the actual baudrate is dictated by the hardware, 
            # but pyserial requires the parameter to be set.
            self.serial_port = serial.Serial(self.port_name, self.baudrate, timeout=1)
            self.connection_status.emit(True, f"Connected to {self.port_name}")
            
            # Clear the input buffer in case of accumulated junk data
            self.serial_port.reset_input_buffer()

            while self.is_running:
                if self.serial_port.in_waiting > 0:
                    # Read the line until \r\n
                    raw_data = self.serial_port.readline()
                    
                    try:
                        # Decode bytes to UTF-8 string
                        text_data = raw_data.decode('utf-8').strip()
                        
                        # Parse the CSV string: "R:0.00,R_ref:0.00,Z:5.00,U1:12.3"
                        # Elegantly convert it into a Python dictionary
                        parsed_dict = {}
                        pairs = text_data.split(',')
                        for pair in pairs:
                            if ':' in pair:
                                key, value = pair.split(':')
                                parsed_dict[key.strip()] = float(value.strip())
                        
                        # Emit the dictionary to the GUI (Thread-safe)
                        if parsed_dict:
                            self.data_updated.emit(parsed_dict)
                            
                    except (UnicodeDecodeError, ValueError) as e:
                        # If a corrupted byte arrives, ignore it and continue.
                        # In critical systems, we don't want the app to crash due to a flipped bit.
                        pass 

        except serial.SerialException as e:
            self.connection_status.emit(False, f"Port Error: {str(e)}")
        finally:
            self.close_port()

    def send_command(self, cmd_char):
        """ Allows injecting commands from the GUI to the STM32 """
        if self.serial_port and self.serial_port.is_open:
            try:
                # Send the character (e.g., 'W') encoded as bytes
                self.serial_port.write(cmd_char.encode('utf-8'))
            except serial.SerialException:
                pass

    def close_port(self):
        self.is_running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.connection_status.emit(False, "Disconnected")

    def stop(self):
        self.is_running = False
        self.wait() # Wait for the thread to finish safely

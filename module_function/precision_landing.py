import json
import socket
from PyQt6.QtWidgets import QVBoxLayout, QLabel, QFrame
from PyQt6.QtCore import QThread, pyqtSignal

class PLReceiverThread(QThread):
    data_received = pyqtSignal(dict)

    def __init__(self, port=5005):
        super().__init__()
        self.port = port
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(1.0)

    def run(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                json_data = json.loads(data.decode('utf-8'))
                self.data_received.emit(json_data)
            except socket.timeout:
                continue
            except json.JSONDecodeError:
                pass 
            except Exception as e:
                print(f"UDP Error: {e}")

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

class PrecisionLandingWidget(QFrame):
    def __init__(self, port=5005, parent=None):
        super().__init__(parent)
        self.port = port
        self.init_ui()
        
        self.receiver_thread = PLReceiverThread(port=self.port)
        self.receiver_thread.data_received.connect(self.update_data)
        self.receiver_thread.start()

    def init_ui(self):
        self.setStyleSheet("background-color: #2c3e50; border-radius: 5px; padding: 5px;")
        layout = QVBoxLayout(self)
        
        layout.addWidget(QLabel("PRECISION LANDING STATUS", styleSheet="font-weight: bold; color: #3498db;"))
        
        self.lbl_target_status = QLabel("Target: NOT DETECTED")
        self.lbl_target_status.setStyleSheet("color: #e74c3c; font-weight: bold;")
        self.lbl_target_pos = QLabel("Pos (x, y): 0.0, 0.0")
        self.lbl_target_pos.setStyleSheet("color: white;")
        self.lbl_target_dist = QLabel("Distance: 0.0 m")
        self.lbl_target_dist.setStyleSheet("color: white;")
        
        layout.addWidget(self.lbl_target_status)
        layout.addWidget(self.lbl_target_pos)
        layout.addWidget(self.lbl_target_dist)

    def update_data(self, data):
        is_tracking = data.get("tracking", False)
        
        if is_tracking:
            self.lbl_target_status.setText("Target: TRACKING")
            self.lbl_target_status.setStyleSheet("color: #2ecc71; font-weight: bold;") 
            
            x, y, dist = data.get("x", 0.0), data.get("y", 0.0), data.get("dist", 0.0)
            self.lbl_target_pos.setText(f"Pos (x, y): {x:.2f}, {y:.2f} m")
            self.lbl_target_dist.setText(f"Distance: {dist:.2f} m")
        else:
            self.lbl_target_status.setText("Target: LOST")
            self.lbl_target_status.setStyleSheet("color: #e74c3c; font-weight: bold;")

    def closeEvent(self, event):
        self.receiver_thread.stop()
        super().closeEvent(event)
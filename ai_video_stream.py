import cv2
import numpy as np
import socket
import struct
import json
import time
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QGroupBox, QFrame)
from PyQt6 import QtGui

# ==============================================================================
# 1. THREAD RECEIVES VIDEO FROM DRONE VIA UDP 
class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(object)

    def __init__(self, port=9999):
        super().__init__()
        self.port = port
        self.running = True

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("0.0.0.0", self.port))
        except Exception as e:
            print(f"PORT ERROR: The {self.port} port is being hijacked by Mavlink! {e}")
            return

        print(f"[UDP] Waiting for drone photos at the gate {self.port}...")

        while self.running:
            try:
                data, _ = sock.recvfrom(65535)
                
                frame_np = np.frombuffer(data, dtype=np.uint8)
                frame = cv2.imdecode(frame_np, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    self.change_pixmap_signal.emit(frame)
            except Exception as e:
                print(f"Packet reception error: {e}")
                continue
        
        sock.close()

    def stop(self):
        self.running = False
        self.wait()

# ==============================================================================
# 2. THREAD RECEIVES DATA PRECISION LANDING 
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
            except:
                continue

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

# ==============================================================================
# 3. MAIN WIDGET
class AIVideoStreamTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.video_thread = None
        self.pl_thread = None
        # IP RASPBERRY PI
        self.PI_IP = "100.112.194.109" 
        self.init_ui()

    def init_ui(self):
        main_layout = QHBoxLayout(self)

        self.video_label = QLabel("NO VIDEO SIGNAL")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setStyleSheet("background-color: black; color: #e74c3c; font-size: 24px; font-weight: bold; border-radius: 10px;")
        self.video_label.setMinimumSize(800, 600) 
        main_layout.addWidget(self.video_label, stretch=3)

        right_panel = QFrame()
        right_panel.setFixedWidth(350)
        right_v_layout = QVBoxLayout(right_panel)
        
        group_cam = QGroupBox("1. CONNECT CAMERA (UDP)")
        group_cam.setStyleSheet("color: white; font-weight: bold;")
        lay_cam = QHBoxLayout()
        self.btn_turn_on = QPushButton("TURN ON STREAM")
        self.btn_turn_on.clicked.connect(self.start_video_stream)
        self.btn_turn_off = QPushButton("TURN OFF")
        self.btn_turn_off.clicked.connect(self.stop_video_stream)
        lay_cam.addWidget(self.btn_turn_on)
        lay_cam.addWidget(self.btn_turn_off)
        group_cam.setLayout(lay_cam)
        right_v_layout.addWidget(group_cam)

        # Precision Landing
        self.group_pl = QGroupBox("4. PRECISION LANDING")
        self.group_pl.setStyleSheet("color: white; font-weight: bold; border: 1px solid #555; margin-top: 10px;")
        pl_layout = QVBoxLayout()
        self.btn_pl_toggle = QPushButton("ENABLE PRECISION LANDING")
        self.btn_pl_toggle.setCheckable(True)
        self.btn_pl_toggle.clicked.connect(self.toggle_precision_landing)
        self.lbl_pl_status = QLabel("Status: STANDBY")
        self.lbl_pl_data = QLabel("X: 0.00 | Y: 0.00 | Dist: 0.00m")
        pl_layout.addWidget(self.btn_pl_toggle)
        pl_layout.addWidget(self.lbl_pl_status)
        pl_layout.addWidget(self.lbl_pl_data)
        self.group_pl.setLayout(pl_layout)
        right_v_layout.addWidget(self.group_pl)

        right_v_layout.addStretch()
        main_layout.addWidget(right_panel, stretch=1)

    def start_video_stream(self):
        if self.video_thread is None:
            self.video_thread = VideoThread(port=9999)
            self.video_thread.change_pixmap_signal.connect(self.update_image)
            self.video_thread.start()
        
        self.btn_turn_on.setStyleSheet("background-color: #2ecc71; color: white;")
        
        try:
            udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_sock.sendto(b"START_STREAM", (self.PI_IP, 8888))
        except Exception as e:
            print(f"Error sending the START command: {e}")

    def stop_video_stream(self):
        try:
            udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_sock.sendto(b"STOP_STREAM", (self.PI_IP, 8888))
        except Exception as e:
            print(f"Error sending STOP command: {e}")

        if self.video_thread:
            self.video_thread.stop()
            self.video_thread = None
        
        self.video_label.clear()
        self.video_label.setText("NO VIDEO SIGNAL")
        self.btn_turn_on.setStyleSheet("")

    def update_image(self, cv_img):
        try:
            qt_img = self.convert_cv_qt(cv_img)
            self.video_label.setPixmap(qt_img)
        except Exception as e:
            print(f"UI ERROR: {e}")

    def convert_cv_qt(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.video_label.width(), self.video_label.height(), Qt.AspectRatioMode.KeepAspectRatio)
        return QtGui.QPixmap.fromImage(p)

    def toggle_precision_landing(self, checked):
        if checked:
            self.pl_thread = PLReceiverThread(port=5005)
            self.pl_thread.data_received.connect(self.update_pl_ui)
            self.pl_thread.start()
        else:
            if self.pl_thread:
                self.pl_thread.stop()
                self.pl_thread = None

    def update_pl_ui(self, data):
        if data.get("tracking", False):
            self.lbl_pl_status.setText("Status: TARGET LOCKED")
            self.lbl_pl_status.setStyleSheet("color: #2ecc71; font-weight: bold;")
            x, y, dist = data.get("x",0), data.get("y",0), data.get("dist",0)
            self.lbl_pl_data.setText(f"X: {x:.2f} | Y: {y:.2f} | Dist: {dist:.2f}m")
        else:
            self.lbl_pl_status.setText("Status: TARGET LOST")
            self.lbl_pl_status.setStyleSheet("color: #e74c3c; font-weight: bold;")

    def closeEvent(self, event):
        self.stop_video_stream()
        if self.pl_thread: self.pl_thread.stop()
        super().closeEvent(event)
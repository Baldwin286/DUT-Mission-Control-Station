import sys
import math
import os
import cv2
import numpy as np
import time
import threading
import socket
import csv

# --- GPU and FFMPEG OPTIMIZED ---
os.environ["QT_OPENGL"] = "desktop" 
os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = "--disable-gpu-compositing --ignore-gpu-blocklist"
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "fflags;nobuffer|flags;low_delay|er;1" 

import serial.tools.list_ports
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QFrame, QPushButton, QComboBox, 
                             QGridLayout, QTextEdit, QGroupBox, QStackedWidget,
                             QMessageBox, QDoubleSpinBox, QSizePolicy)
from PyQt6.QtGui import QPainter, QColor, QPen, QPixmap, QPolygonF, QFont, QImage, QTextCursor
from PyQt6.QtCore import QThread, pyqtSignal, Qt, QPointF, QRectF, QDateTime, QTimer
from PyQt6.QtWebEngineWidgets import QWebEngineView
import pyqtgraph as pg
from pymavlink import mavutil

# --- EMBEDD FLY PROGRAMMING FILE ---
from mission_planner import MissionPlannerApp

# ====================================================================================
# --- CLASS RECEIVES GSTREAMER/FFMPEG VIDEOS ---
class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(object)

    def __init__(self, port=9999):
        super().__init__()
        self.port = port
        self.running = True
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # --- MODULE MANAGEMENT VARIABLES FOR PROCESSING & RECORDING ---
        self.processor = None # By default, no processor is installed
        self.is_recording = False
        self.video_writer = None
        self.record_filename = ""

    def set_processor(self, processor_instance):
        """Receive the processing class from the Main File"""
        self.processor = processor_instance

    def start_recording(self, filename):
        self.record_filename = filename
        self.is_recording = True
        self.video_writer = None

    def stop_recording(self):
        self.is_recording = False
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None

    def run(self):
        video_url = f"udp://@0.0.0.0:{self.port}?overrun_nonfatal=1&fifo_size=50000000&timeout=2000000"
        print("\n[AI STREAM] Waiting for connection from Camera...")

        # LOOP: Always waiting for videos
        while self.running:
            cap = cv2.VideoCapture(video_url, cv2.CAP_FFMPEG)
            
            # NIf there is no video (camera is off), wait 1 second and try again
            if not cap.isOpened():
                self.msleep(1000)
                continue

            self.capture_active = True
            print("\n[AI STREAM] The video stream has been captured!")
            
            # Buffer cleanup stream
            def clear_buffer():
                while self.running and self.capture_active:
                    ret, frame = cap.read()
                    if ret:
                        with self.frame_lock:
                            self.latest_frame = frame 
                    else:
                        time.sleep(0.01)

            reader_thread = threading.Thread(target=clear_buffer, daemon=True)
            reader_thread.start()

            empty_frames = 0
            
            # Main image processing flow
            while self.running and self.capture_active:
                with self.frame_lock:
                    frame_to_process = self.latest_frame
                    self.latest_frame = None 

                if frame_to_process is not None:
                    empty_frames = 0 
                    
                    # 1. PUSH THE FRAME THROUGH THE MODULE FOR PROCESSING (RGB / GRAY / AI)
                    if self.processor is not None:
                        final_frame = self.processor.process(frame_to_process)
                    else:
                        final_frame = frame_to_process.copy()

                    # 2. RECORD VIDEO TO FILE
                    if self.is_recording:
                        if self.video_writer is None:
                            h, w, _ = final_frame.shape
                            fourcc = cv2.VideoWriter_fourcc(*'XVID')
                            self.video_writer = cv2.VideoWriter(self.record_filename, fourcc, 15.0, (w, h))
                        self.video_writer.write(final_frame)

                    # 3. SUBMIT TO MCS UI
                    self.change_pixmap_signal.emit(final_frame)
                else:
                    empty_frames += 1
                    self.msleep(50) 
                    
                    # IF PRESS THE CAMERA OFF BUT IT TAKES MORE THAN 2 SECONDS -> DISCONNECT, COME BACK AND WAIT
                    if empty_frames > 40:
                        print("[AI STREAM] The camera is off. Waiting for a new stream...")
                        self.capture_active = False 
                        break 

            # Perform a cleanup before accepting a new stream
            self.stop_recording()
            cap.release()
            reader_thread.join(timeout=1.0)

    def stop(self):
        self.running = False
        self.terminate() 
        self.wait()
# ====================================================================================

class MavlinkWorker(QThread):
    data_changed = pyqtSignal(dict)
    
    def __init__(self, conn_string, baudrate=115200):
        super().__init__()
        self.conn_string = conn_string
        self.baudrate = baudrate
        self.running = True
        self.connection = None

    def stop(self):
        self.running = False
        if self.connection:
            try: self.connection.close()
            except: pass
        self.wait()

    def run(self):
        try:
            self.connection = mavutil.mavlink_connection(self.conn_string, baud=self.baudrate)
            try:
                self.connection.wait_heartbeat(timeout=3)
                self.connection.mav.request_data_stream_send(
                    self.connection.target_system, self.connection.target_component, 
                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1
                )
            except: pass

            while self.running:
                msg = self.connection.recv_match(blocking=True, timeout=0.5)
                if not msg: continue
                
                t = msg.get_type()
                src_sys = msg.get_srcSystem()
                d = {"type": t}
                
                if t == 'ATTITUDE':
                    d.update({"r": math.degrees(msg.roll), "p": math.degrees(msg.pitch), "y": math.degrees(msg.yaw)})
                elif t == 'VFR_HUD':
                    d.update({"alt": msg.alt, "gs": msg.groundspeed, "vs": msg.climb, "hdg": msg.heading})
                elif t == 'SYS_STATUS' and src_sys == 1:
                    ekf_ok = bool(msg.onboard_control_sensors_health & (1 << 11))
                    d.update({"v": msg.voltage_battery/1000.0, "a": msg.current_battery/100.0, "p": msg.battery_remaining, "ekf": ekf_ok})
                elif t == 'GPS_RAW_INT':
                    d.update({"sats": msg.satellites_visible, "lat": msg.lat/1e7, "lon": msg.lon/1e7})
                elif t == 'HEARTBEAT' and src_sys == 1:
                    is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    modes = {0:'STABILIZE', 1:'ACRO', 2:'ALT_HOLD', 3:'AUTO', 4:'GUIDED', 5:'LOITER', 6:'RTL', 7:'CIRCLE', 9:'LAND', 16:'POSHOLD', 17:'BRAKE'}
                    curr_mode = modes.get(msg.custom_mode, f"MODE:{msg.custom_mode}")
                    d.update({"mode": curr_mode, "armed": bool(is_armed)})
                elif t == 'STATUSTEXT':
                    d.update({"status_msg": msg.text})
                elif t == 'RC_CHANNELS':
                    d.update({"chans": [msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw, msg.chan9_raw, msg.chan10_raw, msg.chan11_raw, msg.chan12_raw]})
                elif t == 'NAV_CONTROLLER_OUTPUT':
                    d.update({"wp_dist": msg.wp_dist})
                elif t == 'MISSION_CURRENT':
                    d.update({"seq": msg.seq})
                elif t in ['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK', 'MISSION_COUNT', 'MISSION_ITEM_INT']:
                    d.update({"type": t, "msg_obj": msg})

                self.data_changed.emit(d)
        except Exception as e:
            self.data_changed.emit({"type": "LOG", "msg": f"<b style='color: red;'>Mavlink Error: {str(e)}</b>"})

class PFDWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.roll, self.pitch, self.heading, self.alt, self.speed, self.batt_v, self.batt_a, self.batt_p, self.gps_sats = 0, 0, 0, 0, 0, 0, 0, 0, 0
        self.ekf_status, self.flight_mode, self.status_msg = "---", "OFFLINE", "Ready"
        self.is_armed = False
        self.setMinimumSize(300, 250)

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height(); cx, cy = w/2, h/2

        # Horizon
        p.save(); p.translate(cx, cy); p.rotate(-self.roll); p.translate(0, self.pitch * 3)
        p.fillRect(-w*2, -h*4, w*4, h*4, QColor(30, 144, 255)); p.fillRect(-w*2, 0, w*4, h*4, QColor(139, 69, 19))   
        p.setPen(QPen(Qt.GlobalColor.white, 2))
        for i in range(-60, 61, 10):
            if i != 0: p.drawLine(-30, -i * 5, 30, -i * 5); p.drawText(35, -i * 5 + 5, str(i))
        p.restore()

        # Top Bar
        p.save(); p.setBrush(QColor(0, 0, 0, 180)); p.drawRect(0, 0, w, 35)
        p.setPen(Qt.GlobalColor.white); p.setFont(QFont("Arial", 11, QFont.Weight.Bold))
        p.drawText(QRectF(0, 0, w, 35), Qt.AlignmentFlag.AlignCenter, f"{int(self.heading)}°")
        p.setPen(QColor(255, 242, 0)); p.setFont(QFont("Arial", 13, QFont.Weight.Bold))
        p.drawText(15, 23, self.flight_mode)
        p.restore()

        # Tapes
        p.save(); tape_w, tape_h, tape_y, left_tx, right_tx = 65, h - 100, 45, 10, w - 75
        p.setBrush(QColor(0, 0, 0, 100)); p.drawRect(left_tx, tape_y, tape_w, tape_h); p.drawRect(right_tx, tape_w, tape_w, tape_h) 
        p.setPen(Qt.GlobalColor.white); p.setFont(QFont("Arial", 9))
        def draw_tape_ticks(x_pos, current_val, is_alt):
            p.save(); p.setClipRect(x_pos, tape_y, tape_w, tape_h); cy_tape = tape_y + tape_h / 2
            for v in range(int(current_val - 30), int(current_val + 30)):
                if v % 5 == 0:
                    y_pos = cy_tape + (current_val - v) * 12
                    if is_alt: p.drawLine(x_pos, int(y_pos), x_pos + 10, int(y_pos)); p.drawText(x_pos + 15, int(y_pos) + 4, str(v))
                    else: p.drawLine(x_pos + tape_w, int(y_pos), x_pos + tape_w - 10, int(y_pos)); p.drawText(x_pos + 5, int(y_pos) + 4, str(v))
            p.restore()
        draw_tape_ticks(left_tx, self.speed, False); draw_tape_ticks(right_tx, self.alt, True)

        p.setBrush(Qt.GlobalColor.black); p.drawRect(5, int(cy-15), 75, 30); p.drawRect(w-80, int(cy-15), 75, 30)
        p.setPen(Qt.GlobalColor.green); p.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        p.drawText(QRectF(5, cy-15, 75, 30), Qt.AlignmentFlag.AlignCenter, f"{self.speed:.1f}"); p.drawText(QRectF(w-80, cy-15, 75, 30), Qt.AlignmentFlag.AlignCenter, f"{self.alt:.1f}m")
        p.restore()

        # Status
        p.save(); p.setPen(QPen(QColor(46, 204, 113) if self.is_armed else QColor(231, 76, 60))); p.setFont(QFont("Arial", 32, QFont.Weight.Bold))
        p.drawText(QRectF(0, cy - 80, w, 60), Qt.AlignmentFlag.AlignCenter, "ARMED" if self.is_armed else "DISARMED")
        p.setPen(Qt.GlobalColor.white); p.setFont(QFont("Arial", 11, QFont.Weight.Bold)); p.drawText(QRectF(0, cy + 35, w, 30), Qt.AlignmentFlag.AlignCenter, self.status_msg)
        p.restore()

        # Bottom HUD 
        p.save()
        p.setBrush(QColor(0, 0, 0, 220))
        p.drawRect(0, h-45, w, 45)
        p.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        p.setPen(Qt.GlobalColor.white)

        combined_str = f"Bat: {self.batt_v:.2f}v {DroneApp.get_lipo_s_count(self.batt_v)}  {self.batt_a:.1f}A  {self.batt_p}%    |    EKF: {self.ekf_status}    |    GPS: {self.gps_sats} Sats"
        p.drawText(QRectF(0, h-45, w, 45), Qt.AlignmentFlag.AlignCenter, combined_str)
        p.restore()

        p.setPen(QPen(Qt.GlobalColor.red, 3)); p.drawLine(int(cx-60), int(cy), int(cx-20), int(cy)); p.drawLine(int(cx+20), int(cy), int(cx+60), int(cy))
        p.drawPolygon(QPolygonF([QPointF(cx, cy), QPointF(cx-8, cy+8), QPointF(cx+8, cy+8)]))

class DroneApp(QMainWindow):
    @staticmethod
    def get_lipo_s_count(voltage):
        if voltage < 5.0: return "(USB)"
        s = round(voltage / 3.7)
        return f"({s}S)" if s > 0 else ""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("DUT MISSION CONTROL STATION V3.0")
        self.resize(1280, 720)
        self.setStyleSheet("background-color: #121212; color: white;")
        
        self.mav_worker = None
        self.drone_pos = [16.0748, 108.1498]
        self.r_b, self.p_b, self.y_b = [], [], []
        self.last_mode, self.last_armed, self.last_chans = None, None, None
        
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        self.start_time = QDateTime.currentDateTime()
        
        self.init_ui()
        self.port_timer = QTimer(self); self.port_timer.timeout.connect(self.refresh_ports); self.port_timer.start(1000)

        # --- INITIATING CORE VIDEO RECEIPT FLOW ---
        self.video_thread = VideoThread(port=9999)
        self.video_thread.change_pixmap_signal.connect(self.update_video_frame)
        self.video_thread.start()
        
        # Set the default mode to RGB when open first time
        self.set_video_mode("RGB")

    def send_camera_command(self, command):
        if command == "START_STREAM" and self.mav_worker is None:
            if hasattr(self, 'ai_log'):
                self.ai_log.append("<span style='color: #e74c3c;'>[WARNING] Please connect to the system before turning on the camera!</span>")
            QMessageBox.warning(self, "WARNING", "You must connect to the drone before starting the video stream!")
            return

        # IP RASPBERRY PI
        PI_IP = "192.168.1.22" 
        PI_PORT = 8888
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(command.encode('utf-8'), (PI_IP, PI_PORT))
            
            if command == "START_STREAM":
                self.is_camera_on = True 
                if hasattr(self, 'ai_log'):
                    self.ai_log.append("<span style='color: #2ecc71;'>[CAMERA] Sent command to turn ON the video stream...</span>")
            
            elif command == "STOP_STREAM":
                self.is_camera_on = False 
                self.stop_video_record() 
                
                if hasattr(self, 'video_frame'):
                    self.video_frame.setPixmap(QPixmap()) 
                    self.video_frame.setText("THE CAMERA IS OFF")
                    self.video_frame.setStyleSheet("background: black; color: #7f8c8d; font-size: 24px; font-weight: bold; border: 2px solid #333; border-radius: 10px;")
                if hasattr(self, 'ai_log'):
                    self.ai_log.append("<span style='color: #e74c3c;'>[CAMERA] Sent command to turn OFF the video stream...</span>")
                    
        except Exception as e:
            if hasattr(self, 'ai_log'):
                self.ai_log.append(f"<span style='color: red;'>[CAMERA] Network error: {e}</span>")

    # --- FUNCTION TO CALL MODULE FILES TO CHANGE MODES ---
    def set_video_mode(self, mode):
        if not hasattr(self, 'video_thread'): return
        
        if mode == "RGB":
            from video_type.video_rgb import RGBProcessor
            if not hasattr(self, 'processor_rgb'):
                self.processor_rgb = RGBProcessor()
            self.video_thread.set_processor(self.processor_rgb)
            
        elif mode == "GRAY":
            from video_type.video_gray import GrayProcessor
            if not hasattr(self, 'processor_gray'):
                self.processor_gray = GrayProcessor()
            self.video_thread.set_processor(self.processor_gray)
            
        elif mode == "AI":
            from video_type.video_ai import AIProcessor
            if not hasattr(self, 'processor_ai'):
                if hasattr(self, 'ai_log'):
                    self.ai_log.append("<span style='color: #f39c12;'>[SYSTEM] Loading the AI ​​model into memory, please wait...</span>")
                    QApplication.processEvents() 
                # LINK TO MODEL FILE
                self.processor_ai = AIProcessor(model_path="F:\PBL6\model3\yolo\my_model.pt")
            self.video_thread.set_processor(self.processor_ai)

        if hasattr(self, 'ai_log'):
            self.ai_log.append(f"<span style='color: #3498db;'>[SYSTEM] Display mode has been switched on: {mode}</span>")

    # --- FUNCTIONS FOR RECORDING AND SAVING IN A SEPARATE FOLDER ---
    def start_video_record(self):
        if not getattr(self, 'is_camera_on', False):
            QMessageBox.warning(self, "Warning", "You must TURN ON the camera before you can record!")
            return
            
        # Automatically create the 'video_records' folder if it doesn't already exist
        if not os.path.exists("video_records"):
            os.makedirs("video_records")
            
        filename = f"video_records/video_record_{QDateTime.currentDateTime().toString('yyyyMMdd_hhmmss')}.avi"
        if hasattr(self, 'video_thread'):
            self.video_thread.start_recording(filename)
            self.btn_start_vrec.setStyleSheet("background: #c0392b; font-weight: bold; color: white; padding: 8px; border-radius: 5px;")
            self.btn_start_vrec.setText("RECORDING NOW...")
            if hasattr(self, 'ai_log'):
                self.ai_log.append(f"<b style='color: #e67e22;'>[RECORD] Start recording video to a file: {filename}</b>")

    def stop_video_record(self):
        if hasattr(self, 'video_thread') and self.video_thread.is_recording:
            self.video_thread.stop_recording()
            self.btn_start_vrec.setStyleSheet("background: #34495e; font-weight: bold; color: white; padding: 8px; border-radius: 5px;")
            self.btn_start_vrec.setText("START RECORDING VIDEO")
            if hasattr(self, 'ai_log'):
                self.ai_log.append("<span style='color: #27ae60;'>[RECORD] Video Record successfully saved.</span>")

    def append_log(self, message):
        time_str = QDateTime.currentDateTime().toString("hh:mm:ss")
        fmt_msg = f"<span style='color: #666;'>[{time_str}]</span> {message}"
        self.txt_log.append(fmt_msg); self.txt_log.moveCursor(QTextCursor.MoveOperation.End)
        self.txt_mission_log.append(fmt_msg); self.txt_mission_log.moveCursor(QTextCursor.MoveOperation.End)

    def init_ui(self):
        central = QWidget(); self.setCentralWidget(central); main_layout = QVBoxLayout(central)

        # HEADER
        header = QHBoxLayout(); logos = QHBoxLayout()
        paths = [r"F:\PBL_DATN\DUT_MCS_App\pictures\VN_flag.jpg", r"F:\PBL_DATN\DUT_MCS_App\pictures\image002.jpg"]
        for p in paths:
            lbl = QLabel()
            if os.path.exists(p): lbl.setPixmap(QPixmap(p).scaled(60, 40, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
            logos.addWidget(lbl)
        header.addLayout(logos)
        title = QLabel("DUT MISSION CONTROL STATION"); title.setStyleSheet("font-size: px; font-weight: bold; color: #e74c3c; margin-left: 15px;")
        header.addWidget(title)
        
        nav_layout = QHBoxLayout()
        self.nav_btn_style = "QPushButton { background-color: #2c3e50; color: white; font-weight: bold; padding: 10px 20px; border-radius: 5px; border: 1px solid #34495e; } QPushButton:hover { background-color: #34495e; }"
        self.active_btn_style = "background-color: #3498db; color: white; font-weight: bold; padding: 10px 20px; border-radius: 5px; border: 1px solid #2980b9;"
        self.btn_nav_flight = QPushButton("FLIGHT DATA"); self.btn_nav_mission = QPushButton("MISSION"); self.btn_nav_video = QPushButton("AI VIDEO STREAM")
        for btn, idx in zip([self.btn_nav_flight, self.btn_nav_mission, self.btn_nav_video], [0, 1, 2]):
            btn.setStyleSheet(self.nav_btn_style); btn.clicked.connect(lambda checked, i=idx: self.switch_page(i)); nav_layout.addWidget(btn)
        header.addStretch(); header.addLayout(nav_layout); header.addStretch()
        
        self.combo_type = QComboBox(); self.combo_type.addItems(["UDP (Pi)", "SERIAL"])
        self.combo_port = QComboBox(); self.combo_port.setMinimumWidth(150); self.refresh_ports()
        self.combo_baud = QComboBox(); self.combo_baud.addItems(["115200", "57600", "38400", "9600"])
        self.btn_connect = QPushButton("CONNECT")
        self.btn_connect.setMinimumWidth(130)
        self.btn_connect.setStyleSheet("background-color: #27ae60; font-weight: bold; padding: 10px 20px; border-radius: 5px;")
        self.btn_connect.clicked.connect(self.toggle_connection)
        header.addWidget(self.combo_type); header.addWidget(self.combo_port); header.addWidget(QLabel("Baud:")); header.addWidget(self.combo_baud); header.addWidget(self.btn_connect)
        main_layout.addLayout(header)

        self.stacked_widget = QStackedWidget(); main_layout.addWidget(self.stacked_widget)

        # PAGE 0: FLIGHT DATA
        self.page_flight = QWidget(); flight_layout = QVBoxLayout(self.page_flight)
        
        top_dash = QHBoxLayout()
        cmd_group = QGroupBox("QUICK ACTIONS")
        cmd_group.setStyleSheet("QGroupBox { font-weight: bold; color: #3498db; border: 1px solid #333; border-radius: 5px; margin-top: 12px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }")
        cmd_lay = QHBoxLayout(cmd_group)
        self.btn_arm = QPushButton("ARM"); self.btn_arm.setStyleSheet("background: #c0392b; font-weight: bold; color: white; padding: 5px;")
        self.btn_disarm = QPushButton("DISARM"); self.btn_disarm.setStyleSheet("background: #7f8c8d; font-weight: bold; color: white; padding: 5px;")
        self.spin_alt = QDoubleSpinBox(); self.spin_alt.setRange(0, 100); self.spin_alt.setValue(10.0); self.spin_alt.setSuffix(" m")
        self.btn_takeoff = QPushButton("TAKEOFF"); self.btn_takeoff.setStyleSheet("background: #27ae60; font-weight: bold; color: white; padding: 5px;")
        self.btn_rtl = QPushButton("RTL"); self.btn_rtl.setStyleSheet("background: #f39c12; font-weight: bold; color: white; padding: 5px;")

        self.btn_arm.clicked.connect(lambda: self.send_long_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1))
        self.btn_disarm.clicked.connect(lambda: self.send_long_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0))
        self.btn_takeoff.clicked.connect(self.action_takeoff)
        self.btn_rtl.clicked.connect(lambda: self.set_mode("RTL"))

        cmd_lay.addWidget(self.btn_arm); cmd_lay.addWidget(self.btn_disarm)
        cmd_lay.addWidget(QLabel("Alt:")); cmd_lay.addWidget(self.spin_alt)
        cmd_lay.addWidget(self.btn_takeoff); cmd_lay.addWidget(self.btn_rtl)
        top_dash.addWidget(cmd_group)

        miss_group = QGroupBox("MISSION PROGRESS")
        miss_group.setStyleSheet("QGroupBox { font-weight: bold; color: #2ecc71; border: 1px solid #333; border-radius: 5px; margin-top: 12px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }")
        miss_lay = QHBoxLayout(miss_group)
        self.lbl_wp = QLabel("Waypoint: -- / --"); self.lbl_wp.setStyleSheet("font-size: 13px; font-weight: bold;")
        self.lbl_time = QLabel("Flight Time: 00:00"); self.lbl_time.setStyleSheet("font-size: 13px; font-weight: bold;")
        miss_lay.addWidget(self.lbl_wp); miss_lay.addWidget(self.lbl_time)
        top_dash.addWidget(miss_group)

        self.btn_record_telemetry = QPushButton("START TELEMETRY LOG")
        self.btn_record_telemetry.setStyleSheet("background: #2c3e50; font-weight: bold; color: white; padding: 15px; border-radius: 5px;")
        self.btn_record_telemetry.clicked.connect(self.toggle_record)
        top_dash.addWidget(self.btn_record_telemetry)

        flight_layout.addLayout(top_dash)

        body = QHBoxLayout(); left_col = QVBoxLayout()
        self.pfd = PFDWidget(); self.pfd.setStyleSheet("border: 2px solid #333; border-radius: 10px;")
        df = QFrame(); df.setStyleSheet("background: #1e1e1e; border-radius: 10px; border: 1px solid #333;"); grid = QGridLayout(df)
        
        def mb(txt, clr):
            l = QLabel("0.00"); l.setMinimumWidth(110); l.setAlignment(Qt.AlignmentFlag.AlignCenter)
            l.setStyleSheet(f"font-size: 28px; font-weight: bold; color: {clr};")
            t = QLabel(txt); t.setStyleSheet("font-size: 10px; color: #888;"); t.setAlignment(Qt.AlignmentFlag.AlignCenter)
            v = QVBoxLayout(); v.addWidget(t); v.addWidget(l); v.setAlignment(l, Qt.AlignmentFlag.AlignCenter); return l, v
        
        self.val_alt, v1 = mb("Altitude (m)", "#d395ff"); self.val_gs, v2 = mb("GroundSpeed (m/s)", "#ffbe76")
        self.val_dist_wp, v3 = mb("Dist to WP (m)", "#ff7979"); self.val_yaw, v4 = mb("Yaw (deg)", "#2ecc71")
        self.val_vs, v5 = mb("Vertical Speed (m/s)", "#f1c40f"); self.val_dist_mav, v6 = mb("DistToMAV", "#3498db")
        grid.addLayout(v1,0,0); grid.addLayout(v2,0,1); grid.addLayout(v3,1,0); grid.addLayout(v4,1,1); grid.addLayout(v5,2,0); grid.addLayout(v6,2,1)
        left_col.addWidget(self.pfd, 3); left_col.addWidget(df, 2)
        right_col = QVBoxLayout(); lg = QGroupBox("SYSTEM LOG MESSAGES"); lg.setStyleSheet("color: #888; font-weight: bold; background: #1a1a1a;"); ll = QVBoxLayout(lg)
        self.txt_log = QTextEdit(); self.txt_log.setReadOnly(True); self.txt_log.setStyleSheet("background: black; color: #2ecc71; font-family: 'Consolas'; font-size: 11px; border: none;"); ll.addWidget(self.txt_log); lg.setMaximumHeight(150)
        
        self.map_view = QWebEngineView(); self.map_view.setStyleSheet("border-radius: 10px; border: 2px solid #333;")
        map_html = """<!DOCTYPE html><html><head><link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/><script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script><style>body,html,#map{margin:0;height:100%;background:#121212;}</style></head><body><div id="map"></div><script>var map=L.map('map',{fadeAnimation:false,zoomAnimation:false,markerZoomAnimation:false}).setView([16.0748,108.1498],17);L.tileLayer('https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png',{maxZoom:20,attribution:'© CartoDB'}).addTo(map);var droneIcon=L.icon({iconUrl:'https://cdn-icons-png.flaticon.com/512/684/684908.png',iconSize:[35,35],iconAnchor:[17,17]});var marker=L.marker([16.0748,108.1498],{icon:droneIcon}).addTo(map);window.updateDronePos=function(lat,lon){var p=[lat,lon];marker.setLatLng(p);var center=map.getCenter();if(map.distance(center,p)>25){map.setView(p,map.getZoom(),{animate:false});}}; </script></body></html>"""
        self.map_view.setHtml(map_html)
        
        right_col.addWidget(lg); right_col.addWidget(self.map_view); body.addLayout(left_col, 1); body.addLayout(right_col, 2); flight_layout.addLayout(body, 5)
        self.graph = pg.PlotWidget(title="REAL-TIME IMU DATA"); self.graph.setBackground('#121212'); self.graph.setMaximumHeight(180); self.graph.setMinimumHeight(100); self.graph.showGrid(x=True, y=True, alpha=0.3); leg = self.graph.addLegend(); leg.anchor((1,0),(1,0),offset=(-10,-16))
        self.c_r = self.graph.plot(pen='r', name="Roll"); self.c_p = self.graph.plot(pen='g', name="Pitch"); self.c_y = self.graph.plot(pen='b', name="Yaw"); flight_layout.addWidget(self.graph)
        self.stacked_widget.addWidget(self.page_flight)

        # PAGE 1: MISSION PLANNER
        self.page_mission_container = QWidget(); mission_container_lay = QVBoxLayout(self.page_mission_container)
        self.page_mission = MissionPlannerApp(self.mav_worker, log_callback=self.append_log)
        mission_container_lay.addWidget(self.page_mission, 4)
        mlg = QGroupBox("MISSION SYSTEM LOG"); mlg.setStyleSheet("color: #888; font-weight: bold; background: #1a1a1a;"); mll = QVBoxLayout(mlg)
        self.txt_mission_log = QTextEdit(); self.txt_mission_log.setReadOnly(True); self.txt_mission_log.setStyleSheet("background: black; color: #3498db; font-family: 'Consolas'; font-size: 11px; border: none;")
        mll.addWidget(self.txt_mission_log); mlg.setMaximumHeight(150); mission_container_lay.addWidget(mlg, 1)
        self.stacked_widget.addWidget(self.page_mission_container)

        # PAGE 2: AI VIDEO
        self.page_video = QWidget(); vl = QHBoxLayout(self.page_video)
        self.video_frame = QLabel("NO VIDEO SIGNAL"); self.video_frame.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_frame.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.video_frame.setStyleSheet("background: black; color: #e74c3c; font-size: 24px; font-weight: bold; border: 2px solid #333; border-radius: 10px;")
        
        ap = QVBoxLayout(); 
        ap.addWidget(QLabel("AI OBJECT DETECTION LOG", styleSheet="font-size: 18px; font-weight: bold; color: #3498db;"))
        
        # --- MODULE 1: PI FLOW CONTROL ---
        group_cam = QGroupBox("1. CONNECT CAMERA")
        group_cam.setStyleSheet("QGroupBox { font-weight: bold; color: #ecf0f1; border: 1px solid #7f8c8d; border-radius: 5px; margin-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }")
        cam_controls = QHBoxLayout(group_cam)
        
        self.btn_start_cam = QPushButton("TURN ON THE CAMERA")
        self.btn_start_cam.setStyleSheet("background: #27ae60; font-weight: bold; color: white; padding: 10px; border-radius: 5px;")
        self.btn_start_cam.clicked.connect(lambda: self.send_camera_command("START_STREAM"))
        
        self.btn_stop_cam = QPushButton("TURN OFF THE CAMERA")
        self.btn_stop_cam.setStyleSheet("background: #c0392b; font-weight: bold; color: white; padding: 10px; border-radius: 5px;")
        self.btn_stop_cam.clicked.connect(lambda: self.send_camera_command("STOP_STREAM"))
        
        cam_controls.addWidget(self.btn_start_cam)
        cam_controls.addWidget(self.btn_stop_cam)
        ap.addWidget(group_cam)

        # --- MODULE 2: DISPLAY MODE ---
        group_mode = QGroupBox("2. VIDEO DISPLAY MODE")
        group_mode.setStyleSheet("QGroupBox { font-weight: bold; color: #ecf0f1; border: 1px solid #7f8c8d; border-radius: 5px; margin-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }")
        mode_controls = QHBoxLayout(group_mode)
        
        self.btn_mode_rgb = QPushButton("RGB NORMAL")
        self.btn_mode_rgb.setStyleSheet("background: #2980b9; font-weight: bold; color: white; padding: 8px; border-radius: 5px;")
        self.btn_mode_rgb.clicked.connect(lambda: self.set_video_mode("RGB"))

        self.btn_mode_gray = QPushButton("GRAYSCALE")
        self.btn_mode_gray.setStyleSheet("background: #7f8c8d; font-weight: bold; color: white; padding: 8px; border-radius: 5px;")
        self.btn_mode_gray.clicked.connect(lambda: self.set_video_mode("GRAY"))

        self.btn_mode_ai = QPushButton("AI DETECT")
        self.btn_mode_ai.setStyleSheet("background: #8e44ad; font-weight: bold; color: white; padding: 8px; border-radius: 5px;")
        self.btn_mode_ai.clicked.connect(lambda: self.set_video_mode("AI"))

        mode_controls.addWidget(self.btn_mode_rgb)
        mode_controls.addWidget(self.btn_mode_gray)
        mode_controls.addWidget(self.btn_mode_ai)
        ap.addWidget(group_mode)

        # --- MODULE 3: VIDEO RECORD CONTROL ---
        group_rec = QGroupBox("3. RECORDING MANAGEMENT")
        group_rec.setStyleSheet("QGroupBox { font-weight: bold; color: #ecf0f1; border: 1px solid #7f8c8d; border-radius: 5px; margin-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }")
        rec_controls = QHBoxLayout(group_rec)
        
        self.btn_start_vrec = QPushButton("START RECORDING VIDEO")
        self.btn_start_vrec.setStyleSheet("background: #34495e; font-weight: bold; color: white; padding: 8px; border-radius: 5px;")
        self.btn_start_vrec.clicked.connect(self.start_video_record)

        self.btn_stop_vrec = QPushButton("STOP RECORDING")
        self.btn_stop_vrec.setStyleSheet("background: #95a5a6; font-weight: bold; color: white; padding: 8px; border-radius: 5px;")
        self.btn_stop_vrec.clicked.connect(self.stop_video_record)

        rec_controls.addWidget(self.btn_start_vrec)
        rec_controls.addWidget(self.btn_stop_vrec)
        ap.addWidget(group_rec)
        # ------------------------------------------

        self.ai_log = QTextEdit("System Ready..."); self.ai_log.setReadOnly(True); self.ai_log.setStyleSheet("background: #1e1e1e; color: #2ecc71; font-family: 'Consolas'; font-size: 12px; border: 1px solid #333; border-radius: 5px;")
        ap.addWidget(self.ai_log); vl.addWidget(self.video_frame, 3); vl.addLayout(ap, 1); self.stacked_widget.addWidget(self.page_video)

        self.switch_page(0)

    def toggle_record(self):
        if not self.is_recording:
            filename = f"telemetry_{QDateTime.currentDateTime().toString('yyyyMMdd_hhmmss')}.csv"
            try:
                self.csv_file = open(filename, 'w', newline='')
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(['Timestamp', 'Mode', 'Armed', 'Alt', 'Spd', 'Yaw', 'Lat', 'Lon', 'Volt'])
                self.is_recording = True
                self.btn_record_telemetry.setText("STOP TELEMETRY LOG")
                self.btn_record_telemetry.setStyleSheet("background: #e74c3c; font-weight: bold; color: white; padding: 15px; border-radius: 5px;")
                self.append_log(f"Recording to {filename}")
            except Exception as e:
                self.append_log(f"CSV Error: {e}")
        else:
            self.is_recording = False
            if self.csv_file: self.csv_file.close()
            self.btn_record_telemetry.setText("START TELEMETRY LOG")
            self.btn_record_telemetry.setStyleSheet("background: #2c3e50; font-weight: bold; color: white; padding: 15px; border-radius: 5px;")
            self.append_log("Recording stopped.")

    def action_takeoff(self):
        alt = self.spin_alt.value()
        if not self.pfd.is_armed:
            QMessageBox.warning(self, "Warning", "Drone DISARMED!")
            return
        self.set_mode("GUIDED")
        QTimer.singleShot(500, lambda: self.send_long_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt))
        self.append_log(f"Takeoff command sent: {alt}m")

    def send_long_cmd(self, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
        if self.mav_worker and self.mav_worker.connection:
            self.mav_worker.connection.mav.command_long_send(
                self.mav_worker.connection.target_system,
                self.mav_worker.connection.target_component,
                command, 0, p1, p2, p3, p4, p5, p6, p7
            )

    def set_mode(self, mode_name):
        if self.mav_worker and self.mav_worker.connection:
            mode_id = self.mav_worker.connection.mode_mapping().get(mode_name)
            if mode_id is not None:
                self.mav_worker.connection.set_mode(mode_id)
                self.append_log(f"Setting Mode: {mode_name}")

    def switch_page(self, index):
        self.stacked_widget.setCurrentIndex(index)
        self.btn_nav_flight.setStyleSheet(self.active_btn_style if index == 0 else self.nav_btn_style)
        self.btn_nav_mission.setStyleSheet(self.active_btn_style if index == 1 else self.nav_btn_style)
        self.btn_nav_video.setStyleSheet(self.active_btn_style if index == 2 else self.nav_btn_style)

        if index != 2 and hasattr(self, 'video_frame'):
            self.video_frame.setPixmap(QPixmap()) 
            self.video_frame.setText("NO VIDEO SIGNAL")

    def refresh_ports(self):
        new_ports = []
        fc_keywords = ["ardupilot", "px4", "mavlink", "cube", "pixhawk", "holybro", "stm", "mamba", "matek", "ch340", "cp210"]
        for p in serial.tools.list_ports.comports():
            desc = (p.description or "").lower()
            if any(kw in desc for kw in fc_keywords): new_ports.append(f"{p.device} (FC)")
            else: new_ports.append(f"{p.device}")
        
        curr = self.combo_port.currentText(); self.combo_port.clear(); self.combo_port.addItems(new_ports)
        if curr in new_ports: self.combo_port.setCurrentText(curr)
        elif new_ports:
            for p in new_ports:
                if "(FC)" in p: self.combo_port.setCurrentText(p); break

    def toggle_connection(self):
        if self.mav_worker and self.mav_worker.isRunning():
            self.mav_worker.stop(); self.mav_worker = None; self.page_mission.mav_worker = None
            self.btn_connect.setText("CONNECT"); self.btn_connect.setStyleSheet("background-color: #27ae60; font-weight: bold; padding: 10px 20px; border-radius: 5px;")
            self.pfd.flight_mode = "OFFLINE"; self.pfd.update()
            
            self.send_camera_command("STOP_STREAM")
            
            self.append_log("<b style='color: red;'>System Offline.</b>"); self.port_timer.start(1000)
        else:
            baud = int(self.combo_baud.currentText()) 
            if self.combo_type.currentText().startswith("UDP"): conn = 'udp:0.0.0.0:14552'
            else:
                full = self.combo_port.currentText()
                if not full: self.append_log("Select COM port!"); return
                conn = full.split()[0].strip()
            self.port_timer.stop()
            self.append_log(f"Connecting to <b style='color: #3498db;'>{conn}</b>...")
            
            self.pfd.flight_mode = "WAITING..."; self.pfd.update()
            
            self.mav_worker = MavlinkWorker(conn, baudrate=baud); self.page_mission.mav_worker = self.mav_worker
            self.mav_worker.data_changed.connect(self.update_ui); self.mav_worker.start()
            self.btn_connect.setText("DISCONNECT"); self.btn_connect.setStyleSheet("background-color: #e74c3c; font-weight: bold; padding: 10px 20px; border-radius: 5px;")

    def update_map(self):
        lat, lon = self.drone_pos
        self.map_view.page().runJavaScript(f"updateDronePos({lat}, {lon});")

    def update_ui(self, data):
        t = data["type"]
        
        if self.is_recording and t == 'VFR_HUD':
            ts = QDateTime.currentDateTime().toString("hh:mm:ss.zzz")
            self.csv_writer.writerow([ts, self.pfd.flight_mode, self.pfd.is_armed, data['alt'], data['gs'], self.pfd.heading, self.drone_pos[0], self.drone_pos[1], self.pfd.batt_v])
        
        if t == "LOG": self.append_log(data["msg"])
        elif t == 'HEARTBEAT':
            if data['mode'] != self.last_mode:
                self.append_log(f"Mode: <b style='color: white;'>{data['mode']}</b>")
                if self.last_mode is not None: self.pfd.status_msg = "Ready" 
                self.last_mode = data['mode']
            if data['armed'] != self.last_armed:
                self.append_log("ARMED" if data['armed'] else "DISARMED")
                if data['armed']:
                    self.start_time = QDateTime.currentDateTime()
                if self.last_armed is not None: self.pfd.status_msg = "Ready"
                self.last_armed = data['armed']
            
            self.pfd.is_armed = data['armed']; self.pfd.flight_mode = data['mode']; self.pfd.update()
            
        elif t == 'ATTITUDE':
            self.pfd.roll, self.pfd.pitch = data["r"], data["p"]; self.val_yaw.setText(f"{data['y']:.2f}"); self.pfd.update()
            self.r_b.append(data["r"]); self.p_b.append(data["p"]); self.y_b.append(data["y"])
            if len(self.r_b) > 100: [b.pop(0) for b in [self.r_b, self.p_b, self.y_b]]
            self.c_r.setData(self.r_b); self.c_p.setData(self.p_b); self.c_y.setData(self.y_b)
        elif t == 'VFR_HUD':
            self.pfd.alt, self.pfd.speed, self.pfd.heading = data['alt'], data['gs'], data['hdg']
            self.val_alt.setText(f"{data['alt']:.1f}"); self.val_gs.setText(f"{data['gs']:.2f}"); self.val_vs.setText(f"{data['vs']:.2f}")
        elif t == 'SYS_STATUS':
            self.pfd.batt_v, self.pfd.batt_a, self.pfd.batt_p = data['v'], data['a'], data['p']
            self.pfd.ekf_status = "OK" if data['ekf'] else "BAD"
        elif t == 'STATUSTEXT':
            self.append_log(f"-> {data['status_msg']}")
            self.pfd.status_msg = data['status_msg']; self.pfd.update() 
            QTimer.singleShot(5000, lambda: setattr(self.pfd, 'status_msg', 'Ready'))
            QTimer.singleShot(5005, self.pfd.update)
        elif t == 'GPS_RAW_INT':
            self.pfd.gps_sats = data['sats']
            self.drone_pos = [data['lat'], data['lon']]; self.update_map()
        elif t == 'RC_CHANNELS':
            if self.last_chans is not None:
                for old, new in zip(self.last_chans, data["chans"]):
                    if 0 < old < 65000 and 0 < new < 65000 and abs(old - new) > 100:
                        if self.pfd.status_msg != "Ready": self.pfd.status_msg = "Ready"; self.pfd.update()
                        break 
            self.last_chans = data["chans"]
        
        elif t == 'NAV_CONTROLLER_OUTPUT':
            self.val_dist_wp.setText(f"{data['wp_dist']:.1f}")
        elif t == 'MISSION_CURRENT':
            self.lbl_wp.setText(f"Waypoint: {data['seq']} / --")
            
        elif t in ['MISSION_REQUEST', 'MISSION_ACK', 'MISSION_COUNT', 'MISSION_ITEM_INT']:
            self.page_mission.handle_mission_msg(data)
            
        if self.pfd.is_armed:
            secs = self.start_time.secsTo(QDateTime.currentDateTime())
            self.lbl_time.setText(f"Flight Time: {secs//60:02d}:{secs%60:02d}")

    def update_video_frame(self, cv_img):
        if not getattr(self, 'is_camera_on', False):
            return 
            
        qt_img = self.convert_cv_qt(cv_img)
        self.video_frame.setPixmap(qt_img)

    def convert_cv_qt(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        
        qt_img = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
        
        lbl_w = self.video_frame.width()
        lbl_h = self.video_frame.height()
        
        if lbl_w > 10 and lbl_h > 10:
            p = qt_img.scaled(
                lbl_w, lbl_h, 
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
            return QPixmap.fromImage(p)
        return QPixmap.fromImage(qt_img)
        
    def closeEvent(self, event):
        self.stop_video_record() 
        self.send_camera_command("STOP_STREAM")
        if hasattr(self, 'video_thread'):
            self.video_thread.stop()
        if self.mav_worker:
            self.mav_worker.stop()
        event.accept()

if __name__ == "__main__":
    try:
        import ctypes
        myappid = 'dut.mcs.pbl.1.0' 
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)
    except:
        pass

    app = QApplication(sys.argv)
    
    # --- 1. SET ICON ---
    from PyQt6.QtGui import QIcon, QPixmap
    from PyQt6.QtWidgets import QSplashScreen
    
    app_icon = QIcon(r"F:\PBL_DATN\DUT_MCS_App\pictures\logo.png") 
    app.setWindowIcon(app_icon)

    # --- 2. SPLASH SCREEN ---
    splash_pix = QPixmap(r"F:\PBL_DATN\DUT_MCS_App\pictures\logo.png") 
    splash_pix = splash_pix.scaled(500, 500, Qt.AspectRatioMode.KeepAspectRatioByExpanding, Qt.TransformationMode.SmoothTransformation)
    
    splash = QSplashScreen(splash_pix, Qt.WindowType.WindowStaysOnTopHint)
    splash.show()
    
    splash.showMessage("Loading the DUT Mission Control Station system...\nPlease wait a moment!", 
                       Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignCenter, 
                       Qt.GlobalColor.white)
    
    app.processEvents() 

    # --- 3. INITIALIZE MAIN APP ---
    win = DroneApp()
    win.showMaximized() 
    
    # --- 4. TURN OFF THE SCREEN WHEN THE APP HAS FINISHED OPENING ---
    splash.finish(win)
    
    sys.exit(app.exec())
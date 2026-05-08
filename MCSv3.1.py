import sys
import math
import os
import cv2
import numpy as np
import time
import threading
import socket
import csv
import json
import shutil

# --- GPU and FFMPEG OPTIMIZED ---
os.environ["QSG_RHI_BACKEND"] = "opengl"
os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = "--disable-gpu-compositing --ignore-gpu-blocklist --log-level=3"
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "fflags;nobuffer|flags;low_delay|er;1"

import serial.tools.list_ports
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QFrame, QPushButton, QComboBox, 
                             QGridLayout, QTextEdit, QGroupBox, QStackedWidget,
                             QMessageBox, QDoubleSpinBox, QSizePolicy, 
                             QLineEdit, QTreeWidget, QTreeWidgetItem, QHeaderView, QFileDialog, QDialog, QSplitter, QTabWidget)
from PyQt6.QtGui import QPainter, QColor, QPen, QPixmap, QPolygonF, QFont, QImage, QTextCursor, QVector3D, QBrush
from PyQt6.QtCore import QThread, pyqtSignal, Qt, QPointF, QRectF, QDateTime, QTimer, QUrl, QRect
from PyQt6.QtWebEngineCore import QWebEnginePage, QWebEngineProfile, QWebEngineSettings
from PyQt6.QtWebEngineWidgets import QWebEngineView
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from pymavlink import mavutil

# --- EMBEDD FLY PROGRAMMING FILE ---
from mission_planner import MissionPlannerApp

# --- IMPORT MODULE ---
from module_function.voice_alert import TTSWorker
from module_function.vibration_graph import IMUGraphWidget
from module_function.flight_data_replay import FlightReplayWidget

# ====================================================================================
def load_config(config_file="config.json"):
    if os.path.exists(config_file):
        with open(config_file, 'r', encoding='utf-8') as f:
            return json.load(f)
    else:
        print(f"[WARNING] Not found {config_file}")
        return {}
CONFIG = load_config()
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
        video_url = f"udp://@0.0.0.0:{self.port}?overrun_nonfatal=1&fifo_size=50000000"
        print(f"\n[AI STREAM] Waiting for H264 streaming from Pi at: {video_url}")

        while self.running:
            cap = cv2.VideoCapture(video_url, cv2.CAP_FFMPEG)
            
            if not cap.isOpened():
                self.msleep(1000)
                continue

            self.capture_active = True
            print("\n[AI STREAM] Video stream captured! Uploading to UI...")

            while self.running and self.capture_active:
                ret, frame_to_process = cap.read()
                
                if not ret:
                    self.msleep(5)
                    continue

                if self.processor is not None:
                    final_frame = self.processor.process(frame_to_process)
                else:
                    final_frame = frame_to_process.copy()

                if self.is_recording:
                    if self.video_writer is None:
                        h, w, _ = final_frame.shape
                        fourcc = cv2.VideoWriter_fourcc(*'XVID')
                        self.video_writer = cv2.VideoWriter(self.record_filename, fourcc, 15.0, (w, h))
                    self.video_writer.write(final_frame)

                self.change_pixmap_signal.emit(final_frame)

            self.stop_recording()
            cap.release()

    def stop(self):
        self.running = False
        self.terminate() 
        self.wait()
# ====================================================================================
# --- GET ACCURATE ESTIMATED PI STOPPING LOCATION ---
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
        print(f"[PL] Listening to Aruco coordinates on UDP {self.port}...")
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                json_data = json.loads(data.decode('utf-8'))
                self.data_received.emit(json_data)
            except socket.timeout:
                continue
            except Exception as e:
                pass

    def stop(self):
        self.running = False
        self.wait()
# ====================================================================================
class MavlinkWorker(QThread):
    data_changed = pyqtSignal(dict)
    connection_failed = pyqtSignal(str) 
    link_status_changed = pyqtSignal(bool)
    
    def __init__(self, conn_string, baudrate=115200):
        super().__init__()
        self.conn_string = conn_string
        self.baudrate = baudrate
        self.running = True
        self.connection = None
        self.last_msg_time = 0
        self.is_link_lost = False

        self.last_gcs_heartbeat = 0
        self.version_logged = False
    def stop(self):
        self.running = False
        if self.connection:
            try: self.connection.close()
            except: pass
        self.wait()

    def run(self):
        import socket
        socket.setdefaulttimeout(3.0) 
        
        while self.running:
            try:
                # if self.connection is None:
                #     self.connection = mavutil.mavlink_connection(self.conn_string, baud=self.baudrate)
                #     msg = self.connection.wait_heartbeat(timeout=10)
                #     if not msg:
                #         raise Exception("No heartbeat received (Timeout)")
                if self.connection is None:
                    self.connection = mavutil.mavlink_connection(self.conn_string, baud=self.baudrate)
                    self.connection.source_system = 100
                    start_wait = time.time()
                    heartbeat_found = False
                    while time.time() - start_wait < 5.0:
                        m = self.connection.recv_match(blocking=True, timeout=0.1)
                        if m and m.get_type() == 'HEARTBEAT' and m.get_srcComponent() == 1:
                            self.connection.target_system = m.get_srcSystem()
                            self.connection.target_component = 1
                            heartbeat_found = True
                            break
                            
                    if not heartbeat_found:
                        raise Exception("No heartbeat received (Timeout)")
                    self.connection.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0
                    )
                    self.last_gcs_heartbeat = time.time()
                    self.connection.mav.request_data_stream_send(
                        self.connection.target_system, self.connection.target_component, 
                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1
                    )
                    self.connection.mav.request_data_stream_send(
                        self.connection.target_system, self.connection.target_component, 
                        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 10, 1
                    )
                    self.connection.mav.autopilot_version_request_send(
                        self.connection.target_system, self.connection.target_component
                    )
                    self.connection.mav.param_request_list_send(
                        self.connection.target_system, self.connection.target_component
                    )
                    # ---------------------------------------
                    
                    self.last_msg_time = time.time()
                    if self.is_link_lost:
                        self.is_link_lost = False
                        self.link_status_changed.emit(True)

                    self.connection.mav.request_data_stream_send(
                        self.connection.target_system, self.connection.target_component, 
                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1
                    )
                    self.connection.mav.request_data_stream_send(
                        self.connection.target_system, self.connection.target_component, 
                        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 10, 1
                    )
                    self.connection.mav.command_long_send(
                        self.connection.target_system, 1,
                        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                        148, 0, 0, 0, 0, 0, 0  
                    )
                    self.connection.mav.command_long_send(
                        self.connection.target_system, self.connection.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 241, 100000, 0, 0, 0, 0, 0
                    )
                    self.connection.mav.command_long_send(
                        self.connection.target_system, self.connection.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 124, 500000, 0, 0, 0, 0, 0
                    )
                    
                    self.last_msg_time = time.time()
                    if self.is_link_lost:
                        self.is_link_lost = False
                        self.link_status_changed.emit(True) 
                
                socket.setdefaulttimeout(None)

                msg = self.connection.recv_match(blocking=True, timeout=0.5)
                
                if msg:
                    self.last_msg_time = time.time()
                    if self.is_link_lost:
                        self.is_link_lost = False
                        self.link_status_changed.emit(True)
                        
                    # t = msg.get_type()
                    # src_sys = msg.get_srcSystem()
                    # d = {"type": t}
                    
                    # if t == 'ATTITUDE':
                    #     d.update({"r": math.degrees(msg.roll), "p": math.degrees(msg.pitch), "y": math.degrees(msg.yaw)})
                    # elif t == 'VFR_HUD':
                    #     d.update({"alt": msg.alt, "gs": msg.groundspeed, "vs": msg.climb, "hdg": msg.heading})
                    # elif t == 'SYS_STATUS' and src_sys == 1:
                    #     ekf_ok = bool(msg.onboard_control_sensors_health & (1 << 11))
                    #     rc_ok = bool(msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER)
                    #     d.update({"v": msg.voltage_battery/1000.0, "a": msg.current_battery/100.0, "p": msg.battery_remaining, "ekf": ekf_ok, "rc_ok": rc_ok})
                    # elif t == 'GPS_RAW_INT':
                    #     hdop_val = msg.eph / 100.0 if msg.eph != 65535 else 0.0
                    #     fix_type = msg.fix_type
                    #     d.update({"sats": msg.satellites_visible, "lat": msg.lat/1e7, "lon": msg.lon/1e7, "cog": msg.cog / 100.0, "hdop": hdop_val, "fix_type": fix_type})
                    # elif t == 'GPS2_RAW':
                    #     hdop2_val = msg.eph / 100.0 if msg.eph != 65535 else 0.0
                    #     d.update({"sats2": msg.satellites_visible, "hdop2": hdop2_val})
                    # elif t == 'GLOBAL_POSITION_INT':
                    #     d.update({"rel_alt": msg.relative_alt / 1000.0})
                    # elif t == 'RAW_IMU':
                    #     d.update({"acc_x": msg.xacc, "acc_y": msg.yacc, "acc_z": msg.zacc})
                    # elif t == 'VIBRATION':
                    #     d.update({"vib_x": msg.vibration_x, "vib_y": msg.vibration_y, "vib_z": msg.vibration_z})
                    # elif t == 'SCALED_IMU':  
                    #     d.update({"acc_x": msg.xacc, "acc_y": msg.yacc, "acc_z": msg.zacc})
                    # elif t == 'HEARTBEAT' and src_sys == 1:
                    #     is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                        
                    #     modes = {0:'STABILIZE', 1:'ACRO', 2:'ALT_HOLD', 3:'AUTO', 4:'GUIDED', 5:'LOITER', 6:'RTL', 7:'CIRCLE', 9:'LAND', 16:'POSHOLD', 17:'BRAKE'}
                    #     curr_mode = modes.get(msg.custom_mode, f"MODE:{msg.custom_mode}")
                    #     d.update({"mode": curr_mode, "armed": is_armed}) 
                    t = msg.get_type()
                    src_sys = msg.get_srcSystem()
                    src_comp = msg.get_srcComponent() 
                    d = {"type": t}
                    
                    if t == 'ATTITUDE':
                        d.update({"r": math.degrees(msg.roll), "p": math.degrees(msg.pitch), "y": math.degrees(msg.yaw)})
                    elif t == 'VFR_HUD':
                        d.update({"alt": msg.alt, "gs": msg.groundspeed, "vs": msg.climb, "hdg": msg.heading})
                    elif t == 'SYS_STATUS' and src_comp == 1:
                        ekf_ok = bool(msg.onboard_control_sensors_health & (1 << 11))
                        rc_ok = bool(msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER)
                        d.update({"v": msg.voltage_battery/1000.0, "a": msg.current_battery/100.0, "p": msg.battery_remaining, "ekf": ekf_ok, "rc_ok": rc_ok})
                    elif t == 'GPS_RAW_INT':
                        hdop_val = msg.eph / 100.0 if msg.eph != 65535 else 0.0
                        fix_type = msg.fix_type
                        d.update({"sats": msg.satellites_visible, "lat": msg.lat/1e7, "lon": msg.lon/1e7, "cog": msg.cog / 100.0, "hdop": hdop_val, "fix_type": fix_type})
                    elif t == 'GPS2_RAW':
                        hdop2_val = msg.eph / 100.0 if msg.eph != 65535 else 0.0
                        d.update({"sats2": msg.satellites_visible, "hdop2": hdop2_val})
                    elif t == 'GLOBAL_POSITION_INT':
                        d.update({"rel_alt": msg.relative_alt / 1000.0})
                    elif t == 'RAW_IMU':
                        d.update({"acc_x": msg.xacc, "acc_y": msg.yacc, "acc_z": msg.zacc})
                    elif t == 'VIBRATION':
                        d.update({"vib_x": msg.vibration_x, "vib_y": msg.vibration_y, "vib_z": msg.vibration_z})
                    elif t == 'SCALED_IMU':  
                        d.update({"acc_x": msg.xacc, "acc_y": msg.yacc, "acc_z": msg.zacc})
                    elif t == 'HEARTBEAT' and src_comp == 1:
                        is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                        modes = {0:'STABILIZE', 1:'ACRO', 2:'ALT_HOLD', 3:'AUTO', 4:'GUIDED', 5:'LOITER', 6:'RTL', 7:'CIRCLE', 9:'LAND', 16:'POSHOLD', 17:'BRAKE'}
                        curr_mode = modes.get(msg.custom_mode, f"MODE:{msg.custom_mode}")
                        d.update({"mode": curr_mode, "armed": is_armed})

                    elif t == 'STATUSTEXT':
                        #d.update({"status_msg": msg.text})
                        status_msg = msg.text
                        sys_keywords = ["ArduCopter V", "Matek", "Frame:", "ChibiOS", "RCOut:"]
                        if any(kw in status_msg for kw in sys_keywords):
                            d.update({"type": "LOG", "msg": f"<b style='color: #e67e22;'>[SYS] {status_msg}</b>"})
                        else:
                            d.update({"status_msg": status_msg})
                    elif t == 'RC_CHANNELS':
                        chans = [msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw, msg.chan9_raw, msg.chan10_raw, msg.chan11_raw, msg.chan12_raw]
                        rc_rssi = msg.rssi
                        rssi_percent = 0
                        if rc_rssi != 255 and rc_rssi > 0:
                            rssi_percent = int((rc_rssi / 254.0) * 100)
                        d.update({"chans": chans, "rssi": rssi_percent })
                    elif t == 'NAV_CONTROLLER_OUTPUT':
                        d.update({"wp_dist": msg.wp_dist})
                    elif t == 'MISSION_CURRENT':
                        d.update({"seq": msg.seq})
                    elif t in ['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK', 'MISSION_COUNT', 'MISSION_ITEM_INT']:
                        d.update({"type": t, "msg_obj": msg})
                    elif t == 'PARAM_VALUE':
                        d.update({"param_id": msg.param_id, "param_value": msg.param_value, "param_type": msg.param_type})
                    elif t == 'AUTOPILOT_VERSION':
                        if not getattr(self, 'version_logged', False): 
                            major = (msg.flight_sw_version >> 24) & 0xFF
                            minor = (msg.flight_sw_version >> 16) & 0xFF
                            patch = (msg.flight_sw_version >> 8) & 0xFF
                            d.update({"type": "LOG", "msg": f"<b style='color: #f1c40f;'>[SYS INFO] ArduPilot V{major}.{minor}.{patch}</b>"})
                            self.version_logged = True 
                    self.data_changed.emit(d)

                else:
                    if time.time() - self.last_msg_time > 3.0 and not self.is_link_lost:
                        self.is_link_lost = True
                        self.link_status_changed.emit(False)

            except Exception as e:
                if not self.running:
                    break

                if not self.is_link_lost:
                    self.is_link_lost = True
                    self.link_status_changed.emit(False)
                
                if self.connection:
                    try: self.connection.close()
                    except: pass
                self.connection = None
                self.version_logged = False
                time.sleep(1.0)

class PFDWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.roll, self.pitch, self.heading, self.alt, self.speed, self.batt_v, self.batt_a, self.batt_p, self.gps_sats = 0, 0, 0, 0, 0, 0, 0, 0, 0
        self.ekf_status, self.flight_mode, self.status_msg = "---", "OFFLINE", "Ready"
        self.is_armed = False
        
        self.setMinimumSize(250, 250) 
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self.gps_hdop = 0.0 
        self.gps2_sats = 0
        self.gps2_hdop = 0.0
        self.rssi = 100
        self.gps_fix_type = 0

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
        p.save(); tape_w, tape_h, tape_y, left_tx, right_tx = 65, h - 80, 35, 10, w - 75
        p.setBrush(QColor(0, 0, 0, 100))
        p.drawRect(left_tx, tape_y, tape_w, tape_h) 
        
        p.drawRect(right_tx, tape_y, tape_w, tape_h) 
        
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
        p.save()
        p.setBrush(QColor(0, 0, 0, 220))
        hud_h = 35
        p.drawRect(0, h - hud_h, w, hud_h)
        p.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        p.setPen(Qt.GlobalColor.white)
        fix_dict = {0: "No GPS", 1: "No Fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK Flt", 6: "RTK Fix"}
        fix_str = fix_dict.get(self.gps_fix_type, "Unk")
        combined_str = f"Bat: {self.batt_v:.1f}V  {self.batt_a:.1f}A ({self.batt_p}%)  |  EKF: {self.ekf_status}  |  GPS1: {self.gps_sats} {fix_str} ({self.gps_hdop:.1f}) GPS2: {self.gps2_sats}({self.gps2_hdop:.1f})  |  RSSI: {self.rssi}%"

        from PyQt6.QtGui import QFontMetrics
        font_size = 10
        font = QFont("Arial", font_size, QFont.Weight.Bold)
        fm = QFontMetrics(font)
        while fm.horizontalAdvance(combined_str) > (w - 10) and font_size > 6:
            font_size -= 1
            font.setPointSize(font_size)
            fm = QFontMetrics(font)

        # p.drawText(QRectF(0, h - hud_h, w, hud_h), Qt.AlignmentFlag.AlignCenter, combined_str)
        # p.restore()
        p.setPen(QPen(Qt.GlobalColor.red, 3)); p.drawLine(int(cx-60), int(cy), int(cx-20), int(cy)); p.drawLine(int(cx+20), int(cy), int(cx+60), int(cy))
        p.drawPolygon(QPolygonF([QPointF(cx, cy), QPointF(cx-8, cy+8), QPointF(cx+8, cy+8)]))
        p.setFont(font)
        p.setPen(Qt.GlobalColor.white)
        p.drawText(QRectF(0, h - hud_h, w, hud_h), Qt.AlignmentFlag.AlignCenter, combined_str)
        p.restore()

class drone3DWidget(QWidget):
    def __init__(self, mav_worker=None, log_callback=None):
        super().__init__()
        self.mav_worker = mav_worker
        self.log_callback = log_callback
        layout = QVBoxLayout(self)

        # --- CALIBRATE ---
        btn_lay = QHBoxLayout()
        self.btn_calib = QPushButton("CALIBRATE LEVEL")
        self.btn_calib.setStyleSheet("background: #f39c12; font-weight: bold; color: white; padding: 10px; border-radius: 5px;")
        self.btn_calib.clicked.connect(self.calibrate_level)
        btn_lay.addWidget(self.btn_calib)
        layout.addLayout(btn_lay)

        from PyQt6.QtGui import QVector3D
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=15, elevation=30, azimuth=45)
        self.view.setBackgroundColor('#121212')
        layout.addWidget(self.view, 1)

        grid = gl.GLGridItem()
        grid.setSize(x=20, y=20, z=20)
        grid.setSpacing(x=1, y=1, z=1)
        self.view.addItem(grid)

        arm_len = 3.0
        
        body_box = gl.GLBoxItem(size=QVector3D(1, 1, 0.5), color=(0.2, 0.2, 0.2, 1.0))
        body_box.init_offset = (-0.5, -0.5, -0.25) 
        self.view.addItem(body_box)

        pts = [
            [arm_len, arm_len, 0.0], [-arm_len, -arm_len, 0.0],   
            [-arm_len, arm_len, 0.0], [arm_len, -arm_len, 0.0]    
        ]
        
        self.arms = []
        for p in pts:
            line_pts = np.array([[0,0,0], p])
            p_color = (1.0, 1.0, 1.0, 1.0) if p[1] < 0 else (1.0, 0.0, 0.0, 1.0)
            arm = gl.GLLinePlotItem(pos=line_pts, color=p_color, width=15, antialias=True, mode='lines')
            arm.init_offset = (0.0, 0.0, 0.0)
            self.arms.append(arm)
            self.view.addItem(arm)
        
        self.motors = []
        self.props = []
        
        motor_mesh = gl.MeshData.cylinder(rows=1, cols=16, radius=[0.2, 0.2], length=0.6)
        prop_mesh = gl.MeshData.cylinder(rows=1, cols=20, radius=[0.8, 0.8], length=0.01)

        for p in pts:
            motor = gl.GLMeshItem(meshdata=motor_mesh, color=(0.4, 0.4, 0.4, 1.0), smooth=True)
            motor.init_offset = (float(p[0]), float(p[1]), -0.3)
            self.motors.append(motor)
            self.view.addItem(motor)

            p_color = (1.0, 1.0, 1.0, 0.7) if p[1] < 0 else (1.0, 0.0, 0.0, 0.7)
            prop = gl.GLMeshItem(meshdata=prop_mesh, color=p_color, smooth=True)
            prop.init_offset = (float(p[0]), float(p[1]), 0.3)
            self.props.append(prop)
            self.view.addItem(prop)

        self.drone_model = [body_box] + self.arms + self.motors + self.props
        
        self.update_attitude(0, 0, 0)

    def update_attitude(self, roll, pitch, yaw):
        for item in self.drone_model:
            item.resetTransform()
            
            if hasattr(item, 'init_offset'):
                dx, dy, dz = item.init_offset
                item.translate(dx, dy, dz)
                
            item.rotate(roll, 1, 0, 0)    
            item.rotate(-pitch, 0, 1, 0)   
            item.rotate(-yaw, 0, 0, 1)     

    def calibrate_level(self):
        if not self.mav_worker or not self.mav_worker.connection:
            if self.log_callback: self.log_callback("<b style='color:red;'>[3D] Error: Drone not connected!</b>")
            return
            
        self.mav_worker.connection.mav.command_long_send(
            self.mav_worker.connection.target_system,
            self.mav_worker.connection.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
            0, 0, 0, 0, 1, 0, 0  
        )
        
        if self.log_callback: self.log_callback("<b style='color:#3498db;'>[3D] The Calibrate Level Acceleration command has been sent...</b>")

class FlightWebPage(QWebEnginePage):
    fly_here_signal = pyqtSignal(float, float)

    def acceptNavigationRequest(self, url, _type, isMainFrame):
        if url.scheme() == "flyhere":
            lat = float(url.host())
            lon = float(url.path().strip('/'))
            self.fly_here_signal.emit(lat, lon)
            return False 
        return super().acceptNavigationRequest(url, _type, isMainFrame)

class SlideToUnlockWidget(QWidget):
    unlocked = pyqtSignal()

    def __init__(self, text="SLIDE", handle_color="#c0392b", parent=None):
        super().__init__(parent)
        self.setFixedSize(260, 45) 
        
        self.setStyleSheet("background: transparent;") 
        
        self.handle_color = QColor(handle_color)
        self.bg_color = QColor("#2c3e50")
        self.text = text
        self.value = 0 
        self.is_dragging = False
        self.handle_width = 55 

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        w = self.width()
        h = self.height()

        p.setBrush(QBrush(self.bg_color))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRoundedRect(0, 0, w, h, 5, 5)

        p.setPen(QPen(QColor(150, 150, 150)))
        p.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        p.drawText(0, 0, w, h, Qt.AlignmentFlag.AlignCenter, self.text)

        max_x = w - self.handle_width
        x = int((self.value / 100.0) * max_x)
        
        p.setBrush(QBrush(self.handle_color))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRoundedRect(x, 0, self.handle_width, h, 5, 5)

        p.setPen(QPen(Qt.GlobalColor.white))
        p.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        p.drawText(x, 0, self.handle_width, h, Qt.AlignmentFlag.AlignCenter, "»")
        
        p.end() 

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            max_x = self.width() - self.handle_width
            x = int((self.value / 100.0) * max_x)
            handle_rect = QRect(x, 0, self.handle_width, self.height())
            
            if handle_rect.contains(event.pos()):
                self.is_dragging = True

    def mouseMoveEvent(self, event):
        if self.is_dragging:
            max_x = self.width() - self.handle_width
            x = event.pos().x() - self.handle_width // 2
            
            if x < 0: x = 0
            if x > max_x: x = max_x
            
            self.value = (x / max_x) * 100
            self.update()

    def mouseReleaseEvent(self, event):
        if self.is_dragging:
            self.is_dragging = False
            if self.value > 85: 
                self.value = 100
                self.update()
                self.unlocked.emit()
                QTimer.singleShot(1000, self.reset_slider)
            else:
                self.reset_slider()

    def reset_slider(self):
        self.value = 0
        self.update()


class SlideConfirmDialog(QDialog):
    def __init__(self, action_name, handle_color="#e74c3c", parent=None):
        super().__init__(parent)
        self.setWindowTitle("CONFIRM ORDER")
        self.setFixedSize(300, 130)
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowType.WindowContextHelpButtonHint)
        self.setStyleSheet("background-color: #1e1e1e; color: white;")
        
        layout = QVBoxLayout(self)
        
        lbl = QLabel(f"Confirm order: {action_name}")
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet("font-size: 15px; font-weight: bold; color: #ecf0f1; margin-bottom: 5px; background: transparent;")
        layout.addWidget(lbl)
        
        self.slider = SlideToUnlockWidget(text=f"SLIDE TO {action_name}", handle_color=handle_color)
        self.slider.unlocked.connect(self.accept) 
        
        layout.addWidget(self.slider, alignment=Qt.AlignmentFlag.AlignCenter)

class DroneApp(QMainWindow):
    @staticmethod
    def get_lipo_s_count(voltage):
        if voltage < 5.0: return "(USB)"
        s = round(voltage / 3.7)
        return f"({s}S)" if s > 0 else ""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("DUT MISSION CONTROL STATION V3.1.1")
        self.resize(1280, 720)
        self.setStyleSheet("background-color: #121212; color: white;")
        
        self.mav_worker = None
        self.pl_thread = None
        self.drone_pos = [16.0748, 108.1498]
        self.drone_heading = 0 
        self.drone_course = 0
        self.r_b, self.p_b, self.y_b = [], [], []
        self.last_mode, self.last_armed, self.last_chans = None, None, None
        self.drone_pos = [16.0748, 108.1498]
        self.home_pos = None  # Save the home position when GPS fix is acquired
        
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        self.start_time = QDateTime.currentDateTime()
        self.rc_healthy = True #Check if RC is healthy for TTS alert

        self.total_waypoints = 0
        self.current_wp = 0
        self.flight_timer = QTimer(self)
        self.flight_timer.timeout.connect(self.update_flight_time)
        self.flight_timer.start(1000)

        self.kml_file = None
        self.tts_worker = TTSWorker()
        self.tts_worker.start()

        self.param_groups = {}    
        self.param_tree_items = {}
        
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
        #PI_IP = "192.168.1.22" 
        PI_IP = CONFIG.get("network", {}).get("companion_ip", "100.112.194.109")
        PI_PORT = CONFIG.get("network", {}).get("companion_port", 8888)
        
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
                yolo_path = CONFIG.get("paths", {}).get("model_yolo", "")
                self.processor_ai = AIProcessor(model_path=yolo_path)
            self.video_thread.set_processor(self.processor_ai)

        if hasattr(self, 'ai_log'):
            self.ai_log.append(f"<span style='color: #3498db;'>[SYSTEM] Display mode has been switched on: {mode}</span>")

    # --- FUNCTIONS FOR RECORDING AND SAVING IN A SEPARATE FOLDER ---
    def start_video_record(self):
        if not getattr(self, 'is_camera_on', False):
            QMessageBox.warning(self, "Warning", "You must TURN ON the camera before you can record!")
            return
            
        if not os.path.exists("video_records"):
            os.makedirs("video_records")
            
        filename = f"video_records/video_record_{QDateTime.currentDateTime().toString('yyyyMMdd_hhmmss')}.avi"
        self.current_video_temp_path = filename  
        
        if hasattr(self, 'video_thread'):
            self.video_thread.start_recording(filename)
            self.btn_start_vrec.setStyleSheet("background: #c0392b; font-weight: bold; color: white; padding: 8px; border-radius: 5px;")
            self.btn_start_vrec.setText("RECORDING NOW...")
            if hasattr(self, 'ai_log'):
                self.ai_log.append(f"<b style='color: #e67e22;'>[RECORD] Start recording video...</b>")

    def stop_video_record(self):
        if hasattr(self, 'video_thread') and self.video_thread.is_recording:
            self.video_thread.stop_recording()
            self.btn_start_vrec.setStyleSheet("background: #34495e; font-weight: bold; color: white; padding: 8px; border-radius: 5px;")
            self.btn_start_vrec.setText("START RECORDING VIDEO")
            
            if hasattr(self, 'current_video_temp_path') and os.path.exists(self.current_video_temp_path):
                default_name = os.path.basename(self.current_video_temp_path)
                save_path, _ = QFileDialog.getSaveFileName(
                    self, "Choose a location to save the video", default_name, "AVI Files (*.avi);;All Files (*)"
                )
                if save_path:
                    try:
                        shutil.move(self.current_video_temp_path, save_path)
                        if hasattr(self, 'ai_log'):
                            self.ai_log.append(f"<span style='color: #27ae60;'>[RECORD] The video was saved at: {save_path}</span>")
                    except Exception as e:
                        if hasattr(self, 'ai_log'):
                            self.ai_log.append(f"<span style='color: red;'>[RECORD] Error while saving: {e}</span>")
                else:
                    if hasattr(self, 'ai_log'):
                        self.ai_log.append("<span style='color: #f39c12;'>[RECORD] The transfer has been canceled, but the original file remains in the video_records folder. </span>")

    def append_log(self, message):
        time_str = QDateTime.currentDateTime().toString("hh:mm:ss")
        fmt_msg = f"<span style='color: #666;'>[{time_str}]</span> {message}"
        
        if hasattr(self, 'txt_log'):
            self.txt_log.append(fmt_msg)
            self.txt_log.moveCursor(QTextCursor.MoveOperation.End)
            
        if hasattr(self, 'txt_mission_log'):
            self.txt_mission_log.append(fmt_msg)
            self.txt_mission_log.moveCursor(QTextCursor.MoveOperation.End)
            
        if not hasattr(self, 'txt_log') and not hasattr(self, 'txt_mission_log'):
            print(f"[INIT LOG] {message}")

    # --- FUNCTION TO FLY TO A SPECIFIC LOCATION WHEN CLICKING ON THE MAP ---
    def action_fly_here(self, lat, lon):
        if not self.mav_worker or not self.mav_worker.connection:
            QMessageBox.warning(self, "Warning", "Please connect to the drone first!")
            return
            
        alt = self.spin_alt.value() 
        
        # 1. mode GUIDED
        self.set_mode("GUIDED")
        
        # 2. Send flight command to coordinates (lat, lon, alt)
        self.mav_worker.connection.mav.set_position_target_global_int_send(
            0, # time_boot_ms
            self.mav_worker.connection.target_system,
            self.mav_worker.connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        self.append_log(f"<b style='color:#e74c3c;'>[FLY HERE] Routing to: {lat:.6f}, {lon:.6f} at {alt}m</b>")
        if hasattr(self, 'tts_worker'):
            self.tts_worker.speak("Rerouting to new target")

    def init_ui(self):
        central = QWidget(); self.setCentralWidget(central); main_layout = QVBoxLayout(central)

        # HEADER
        header_v = QVBoxLayout() 

        top_row = QHBoxLayout()
        logos = QHBoxLayout()
        paths = [
            CONFIG.get("paths", {}).get("flag_vn", "VN_flag.jpg"), 
            CONFIG.get("paths", {}).get("flag_dut", "image002.jpg")
        ]
        for p in paths:
            lbl = QLabel()
            if os.path.exists(p): lbl.setPixmap(QPixmap(p).scaled(60, 40, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
            logos.addWidget(lbl)
        top_row.addLayout(logos)
        
        title = QLabel("DUT MISSION CONTROL STATION"); title.setStyleSheet("font-size: 20px; font-weight: bold; color: #e74c3c; margin-left: 15px;")
        top_row.addWidget(title)
        
        top_row.addStretch()
        
        self.combo_type = QComboBox(); self.combo_type.addItems(["SITL (UDP 14550)", "SITL (TCP 5762)", "UDP (Pi)", "SERIAL", "WIFI Local (ESP8266)"])
        self.combo_port = QComboBox(); self.combo_port.setMinimumWidth(150); self.refresh_ports()
        self.combo_baud = QComboBox(); self.combo_baud.addItems(["115200", "57600", "38400", "9600"])
        self.btn_connect = QPushButton("CONNECT")
        self.btn_connect.setMinimumWidth(130)
        self.btn_connect.setStyleSheet("background-color: #27ae60; font-weight: bold; padding: 10px 20px; border-radius: 5px;")
        self.btn_connect.clicked.connect(self.toggle_connection)
        
        top_row.addWidget(self.combo_type); top_row.addWidget(self.combo_port); top_row.addWidget(QLabel("Baud:")); top_row.addWidget(self.combo_baud); top_row.addWidget(self.btn_connect)
        
        header_v.addLayout(top_row)

        nav_row = QHBoxLayout()
        self.nav_btn_style = "QPushButton { background-color: #2c3e50; color: white; font-weight: bold; font-size: 12px; padding: 8px 12px; border-radius: 5px; border: 1px solid #34495e; } QPushButton:hover { background-color: #34495e; }"
        self.active_btn_style = "background-color: #3498db; color: white; font-weight: bold; font-size: 12px; padding: 8px 12px; border-radius: 5px; border: 1px solid #2980b9;"
        
        self.btn_nav_flight = QPushButton("FLIGHT DATA"); self.btn_nav_mission = QPushButton("MISSION"); self.btn_nav_video = QPushButton("AI VIDEO STREAM"); self.btn_nav_replay = QPushButton("FLIGHT REPLAY"); self.btn_nav_setup = QPushButton("3D SETUP")
        self.btn_nav_config = QPushButton("CONFIG")
        btns = [self.btn_nav_flight, self.btn_nav_mission, self.btn_nav_video, self.btn_nav_replay, self.btn_nav_config, self.btn_nav_setup]
        
        nav_row.addStretch() 
        for idx, btn in enumerate(btns):
            btn.setStyleSheet(self.nav_btn_style);btn.setMinimumWidth(110) ;btn.clicked.connect(lambda checked, i=idx: self.switch_page(i)); nav_row.addWidget(btn)
        nav_row.addStretch() 
        
        header_v.addLayout(nav_row)

        main_layout.addLayout(header_v)

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
        self.btn_land = QPushButton("LAND"); self.btn_land.setStyleSheet("background: #2980b9; font-weight: bold; color: white; padding: 5px;")
        self.btn_rtl = QPushButton("RTL"); self.btn_rtl.setStyleSheet("background: #f39c12; font-weight: bold; color: white; padding: 5px;")
        self.btn_loiter = QPushButton("LOITER"); self.btn_loiter.setStyleSheet("background: #8e44ad; font-weight: bold; color: white; padding: 5px;")
        self.btn_loiter.clicked.connect(lambda: self.set_mode("LOITER"))

        action_buttons = [self.btn_arm, self.btn_disarm, self.btn_takeoff, self.btn_loiter, self.btn_land, self.btn_rtl]
        for btn in action_buttons:
            btn.setFixedWidth(64)  

        self.btn_arm.clicked.connect(lambda: self.confirm_and_send_cmd(
            "ARM", "#c0392b", 
            lambda: self.send_long_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        ))
        
        self.btn_disarm.clicked.connect(lambda: self.confirm_and_send_cmd(
            "DISARM", "#7f8c8d", 
            lambda: self.send_long_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        ))
        
        self.btn_takeoff.clicked.connect(lambda: self.confirm_and_send_cmd(
            "TAKEOFF", "#27ae60", 
            self.action_takeoff
        ))
        # self.btn_arm.clicked.connect(lambda: self.send_long_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1))
        # self.btn_disarm.clicked.connect(lambda: self.send_long_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0))
        # self.btn_takeoff.clicked.connect(self.action_takeoff)
        self.btn_land.clicked.connect(lambda: self.set_mode("LAND"))
        self.btn_rtl.clicked.connect(lambda: self.set_mode("RTL"))

        cmd_lay.addWidget(self.btn_arm); cmd_lay.addWidget(self.btn_disarm)
        cmd_lay.addWidget(QLabel("Alt:")); cmd_lay.addWidget(self.spin_alt)
        cmd_lay.addWidget(self.btn_takeoff);cmd_lay.addWidget(self.btn_loiter) ;cmd_lay.addWidget(self.btn_land) ;cmd_lay.addWidget(self.btn_rtl)
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
        left_col.addWidget(self.pfd, 4); left_col.addWidget(df, 1)
        right_col = QVBoxLayout(); lg = QGroupBox("SYSTEM LOG MESSAGES"); lg.setStyleSheet("color: #888; font-weight: bold; background: #1a1a1a;"); ll = QVBoxLayout(lg)
        self.txt_log = QTextEdit(); self.txt_log.setReadOnly(True); self.txt_log.setStyleSheet("background: black; color: #2ecc71; font-family: 'Consolas'; font-size: 11px; border: none;"); ll.addWidget(self.txt_log); lg.setMaximumHeight(150)
        
        self.map_view = QWebEngineView(); self.map_view.setStyleSheet("border-radius: 10px; border: 2px solid #333;")
        self.flight_map_page = FlightWebPage()
        self.flight_map_page.fly_here_signal.connect(self.action_fly_here)
        self.map_view.setPage(self.flight_map_page)
        page = self.map_view.page()
        profile = page.profile()
        user_agent = "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/123.0.0.0 Safari/537.36"
        self.map_view.page().profile().setHttpUserAgent(user_agent)
        self.map_view.page().profile().clearHttpCache()
        profile.setHttpUserAgent(user_agent)
        self.map_view.settings().setAttribute(QWebEngineSettings.WebAttribute.JavascriptEnabled, True)
        self.map_view.settings().setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
        # --- READ HTML MAP FILES ---
        base_dir = os.path.dirname(os.path.abspath(__file__))
        map_path = CONFIG.get("paths", {}).get("map_mcs", os.path.join(base_dir, "map_data", "map_MCS.html")) 
        self.map_view.load(QUrl.fromLocalFile(map_path))
        try:
            with open(map_path, "r", encoding="utf-8") as f:
                map_html = f.read()
        except Exception as e:
            map_html = f"<h1>Map file not found error: {e}</h1>"
            self.append_log(f"<b style='color:red;'>Lỗi tải map: {e}</b>")
            
        self.map_view.setHtml(map_html)
        # ----------------------------------------
        
        right_col.addWidget(lg); right_col.addWidget(self.map_view); body.addLayout(left_col, 1); body.addLayout(right_col, 2); flight_layout.addLayout(body, 5)
        # ---------------------------------------------
    
        # self.graph = pg.PlotWidget(title="REAL-TIME IMU DATA"); self.graph.setBackground('#121212'); self.graph.setMaximumHeight(180); self.graph.setMinimumHeight(100); self.graph.showGrid(x=True, y=True, alpha=0.3); leg = self.graph.addLegend(); leg.anchor((1,0),(1,0),offset=(-10,-16))
        # self.c_r = self.graph.plot(pen='r', name="Roll"); self.c_p = self.graph.plot(pen='g', name="Pitch"); self.c_y = self.graph.plot(pen='b', name="Yaw"); flight_layout.addWidget(self.graph)
        # self.stacked_widget.addWidget(self.page_flight)
        self.imu_graph = IMUGraphWidget()
        flight_layout.addWidget(self.imu_graph)
        self.stacked_widget.addWidget(self.page_flight)

        # PAGE 1: MISSION PLANNER
        self.page_mission_container = QWidget(); mission_container_lay = QVBoxLayout(self.page_mission_container)
        self.page_mission = MissionPlannerApp(self.mav_worker, log_callback=self.append_log)
        self.page_mission.mission_uploaded_signal.connect(self.force_update_wp_count)
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
        # ======================================================================
        # --- MODULE 4: PRECISION LANDING ---
        group_pl = QGroupBox("4. PRECISION LANDING")
        group_pl.setStyleSheet("QGroupBox { font-weight: bold; color: #ecf0f1; border: 1px solid #7f8c8d; border-radius: 5px; margin-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }")
        pl_layout = QVBoxLayout(group_pl)

        self.btn_pl_toggle = QPushButton("ENABLE PRECISION LANDING")
        self.btn_pl_toggle.setCheckable(True)
        self.btn_pl_toggle.setStyleSheet("background: #2c3e50; font-weight: bold; color: white; padding: 8px; border-radius: 5px;")
        self.btn_pl_toggle.clicked.connect(self.toggle_precision_landing)

        self.lbl_pl_status = QLabel("Status: STANDBY")
        self.lbl_pl_status.setStyleSheet("color: #f39c12; font-weight: bold; font-size: 13px;")
        
        self.lbl_pl_data = QLabel("X: 0.00 | Y: 0.00 | Dist: 0.00m")
        self.lbl_pl_data.setStyleSheet("color: #bdc3c7; font-size: 12px;")

        pl_layout.addWidget(self.btn_pl_toggle)
        pl_layout.addWidget(self.lbl_pl_status)
        pl_layout.addWidget(self.lbl_pl_data)
        ap.addWidget(group_pl)
        # ======================================================================

        self.ai_log = QTextEdit("System Ready..."); self.ai_log.setReadOnly(True); self.ai_log.setStyleSheet("background: #1e1e1e; color: #2ecc71; font-family: 'Consolas'; font-size: 12px; border: 1px solid #333; border-radius: 5px;")
        ap.addWidget(self.ai_log); vl.addWidget(self.video_frame, 3); vl.addLayout(ap, 1); self.stacked_widget.addWidget(self.page_video)
    
    # PAGE 3: FLIGHT REPLAY
        self.page_replay = FlightReplayWidget()
        self.stacked_widget.addWidget(self.page_replay)

        # PAGE 4: CONFIG & PARAMETER TUNING
        self.page_config = QWidget()
        cfg_layout = QVBoxLayout(self.page_config)

        # 1. Quick Tuning - WPNAV_SPEED
        quick_group = QGroupBox("QUICK TUNING (AUTO MODE)")
        quick_group.setStyleSheet("""
            QGroupBox { 
                font-weight: bold; 
                color: #f39c12; 
                border: 1px solid #333; 
                border-radius: 5px; 
                margin-top: 15px; 
                padding-top: 15px; 
            } 
            QGroupBox::title { 
                subcontrol-origin: margin; 
                subcontrol-position: top left;
                left: 10px; 
                top: 0px;
                padding: 0 3px; 
            }
        """)
        quick_lay = QHBoxLayout(quick_group)
        
        quick_lay.addWidget(QLabel("Auto Flight Speed (WPNAV_SPEED):"))
        self.spin_wp_speed = QDoubleSpinBox()
        self.spin_wp_speed.setRange(0, 50)
        self.spin_wp_speed.setSuffix(" m/s")
        self.spin_wp_speed.setMinimumWidth(150)
        
        self.btn_set_wp_speed = QPushButton("SET SPEED")
        self.btn_set_wp_speed.setStyleSheet("background: #d35400; font-weight: bold; color: white; padding: 5px;")
        self.btn_set_wp_speed.clicked.connect(lambda: self.write_param("WPNAV_SPEED", self.spin_wp_speed.value() * 100.0))
        
        quick_lay.addWidget(self.spin_wp_speed)
        quick_lay.addWidget(self.btn_set_wp_speed)
        quick_lay.addStretch()
        cfg_layout.addWidget(quick_group)

        self.search_box = QLineEdit()
        self.search_box.setPlaceholderText("Search parameters...")
        self.search_box.setStyleSheet("background: #2c3e50; color: white; padding: 8px; border-radius: 5px; font-weight: bold; border: 1px solid #34495e;")
        self.search_box.textChanged.connect(self.search_params) 
        cfg_layout.addWidget(self.search_box)

        self.param_tree = QTreeWidget()
        self.param_tree.setColumnCount(2)
        self.param_tree.setHeaderLabels(["Parameter Name", "Value"])
        self.param_tree.header().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.param_tree.setStyleSheet("""
            QTreeWidget { background-color: #1e1e1e; color: #2ecc71; border: 1px solid #333; }
            QTreeWidget::item:hover { background-color: #2c3e50; }
            QTreeWidget::item:selected { background-color: #34495e; color: #f1c40f; font-weight: bold; }
        """)
        cfg_layout.addWidget(self.param_tree, 1)

        cfg_btn_lay = QHBoxLayout()
        self.btn_refresh_params = QPushButton("REFRESH ALL PARAMS")
        self.btn_refresh_params.setStyleSheet("background: #2980b9; font-weight: bold; padding: 10px; border-radius: 5px; color: white;")
        self.btn_refresh_params.clicked.connect(self.request_all_params)
        
        self.btn_write_param = QPushButton("WRITE SELECTED PARAM")
        self.btn_write_param.setStyleSheet("background: #27ae60; font-weight: bold; padding: 10px; border-radius: 5px; color: white;")
        self.btn_write_param.clicked.connect(self.action_write_selected_param)
        
        cfg_btn_lay.addWidget(self.btn_refresh_params)
        cfg_btn_lay.addWidget(self.btn_write_param)
        cfg_layout.addLayout(cfg_btn_lay)
        self.stacked_widget.addWidget(self.page_config)

        # PAGE 5: 3D SETUP 
        self.page_setup = drone3DWidget(mav_worker=self.mav_worker, log_callback=self.append_log)
        self.stacked_widget.addWidget(self.page_setup)
        self.switch_page(0)

    def update_flight_time(self):
        if hasattr(self, 'pfd') and self.pfd.is_armed and hasattr(self, 'start_time'):
            elapsed_ms = self.start_time.msecsTo(QDateTime.currentDateTime())
            total_seconds = elapsed_ms // 1000
            minutes = total_seconds // 60
            seconds = total_seconds % 60
            
            if hasattr(self, 'lbl_time'):
                self.lbl_time.setText(f"Flight Time: {minutes:02d}:{seconds:02d}")
        elif hasattr(self, 'pfd') and not self.pfd.is_armed:
            if hasattr(self, 'lbl_time') and self.lbl_time.text() != "Flight Time: 00:00":
                self.lbl_time.setText("Flight Time: 00:00")

    def handle_link_status(self, is_connected):
        if is_connected:
            self.btn_connect.setText("DISCONNECT")
            self.btn_connect.setStyleSheet("background-color: #e74c3c; font-weight: bold; padding: 10px 20px; border-radius: 5px;")
            self.append_log("<b style='color: #2ecc71;'>[SYSTEM] Telemetry Link Recovered!</b>")
            if hasattr(self, 'tts_worker'):
                self.tts_worker.speak("Telemetry link recovered")
        else:
            self.btn_connect.setText("RECONNECTING...")
            self.btn_connect.setStyleSheet("background-color: #f39c12; font-weight: bold; padding: 10px 20px; border-radius: 5px;")
            self.pfd.flight_mode = "LINK LOST"
            self.pfd.update()
            self.append_log("<b style='color: #f39c12;'>[SYSTEM] Telemetry Link Lost! Reconnecting...</b>")
            if hasattr(self, 'tts_worker'):
                self.tts_worker.speak("Warning, telemetry link lost")
    
    def force_update_wp_count(self, new_total):
        self.total_waypoints = new_total
        self.lbl_wp.setText(f"Waypoint: {self.current_wp} / {self.total_waypoints}")
        self.append_log(f"<b style='color:#3498db;'>[MISSION] System updated to {new_total} waypoints.</b>")

    def toggle_record(self):
        if not self.is_recording:
            time_str = QDateTime.currentDateTime().toString('yyyyMMdd_hhmmss')
            
            base_dir = os.path.dirname(os.path.abspath(__file__))
            log_dir = os.path.join(base_dir, "Telemetry_Logs")
            if not os.path.exists(log_dir):
                os.makedirs(log_dir)
                
            self.current_csv_temp_path = os.path.join(log_dir, f"telemetry_{time_str}.csv")
            self.current_kml_temp_path = os.path.join(log_dir, f"trajectory_{time_str}.kml")
            
            try:
                # CSV
                self.csv_file = open(self.current_csv_temp_path, 'w', newline='')
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(['Timestamp', 'Mode', 'Armed', 'Alt', 'Spd', 'Yaw', 'Lat', 'Lon', 'Volt', 'HDOP', 'RSSI'])
                
                # KML
                self.kml_file = open(self.current_kml_temp_path, 'w')
                self.kml_file.write('<?xml version="1.0" encoding="UTF-8"?>\n<kml xmlns="http://www.opengis.net/kml/2.2">\n<Document>\n<name>Flight Trajectory</name>\n<Placemark>\n<LineString>\n<extrude>1</extrude>\n<tessellate>1</tessellate>\n<altitudeMode>relativeToGround</altitudeMode>\n<coordinates>\n')
                
                self.is_recording = True
                self.btn_record_telemetry.setText("STOP TELEMETRY LOG")
                self.btn_record_telemetry.setStyleSheet("background: #e74c3c; font-weight: bold; color: white; padding: 15px; border-radius: 5px;")
                
                self.append_log("Telemetry recording started...")
                if hasattr(self, 'tts_worker'):
                    self.tts_worker.speak("Telemetry recording started")
                    
            except Exception as e:
                self.append_log(f"CSV/KML Error: {e}")
        else:
            self.is_recording = False
            if self.csv_file: self.csv_file.close()
            if self.kml_file:
                self.kml_file.write('\n</coordinates>\n</LineString>\n</Placemark>\n</Document>\n</kml>')
                self.kml_file.close()
                
            self.btn_record_telemetry.setText("START TELEMETRY LOG")
            self.btn_record_telemetry.setStyleSheet("background: #2c3e50; font-weight: bold; color: white; padding: 15px; border-radius: 5px;")
            
            folder_path = QFileDialog.getExistingDirectory(self, "Choose a folder to save the Telemetry files")
            
            if folder_path:
                try:
                    new_csv = os.path.join(folder_path, os.path.basename(self.current_csv_temp_path))
                    new_kml = os.path.join(folder_path, os.path.basename(self.current_kml_temp_path))
                    
                    shutil.move(self.current_csv_temp_path, new_csv)
                    shutil.move(self.current_kml_temp_path, new_kml)
                    self.append_log(f"The Telemetry file has been moved: <b style='color:#f39c12;'>{folder_path}</b>")
                except Exception as e:
                    self.append_log(f"Error saving telemetry file: {e}")
            else:
                self.append_log("Even after de-saving, temporary files remain in the Telemetry_Logs folder.")

            if hasattr(self, 'tts_worker'):
                self.tts_worker.speak("Telemetry recording stopped")

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
        self.btn_nav_replay.setStyleSheet(self.active_btn_style if index == 3 else self.nav_btn_style)
        self.btn_nav_config.setStyleSheet(self.active_btn_style if index == 4 else self.nav_btn_style)
        self.btn_nav_setup.setStyleSheet(self.active_btn_style if index == 5 else self.nav_btn_style)

        if index != 2 and hasattr(self, 'video_frame'):
            self.video_frame.setPixmap(QPixmap()) 
            self.video_frame.setText("NO VIDEO SIGNAL")

    # ======================================================================
    # --- PRECISION LANDING HANDLER ---
    def toggle_precision_landing(self, checked):
        if checked:
            if not self.mav_worker or not self.mav_worker.connection:
                QMessageBox.warning(self, "Warning", "Please connect the system before enabling the Landing function!")
                self.btn_pl_toggle.setChecked(False)
                return
                
            self.btn_pl_toggle.setText("DISABLE PRECISION LANDING")
            self.btn_pl_toggle.setStyleSheet("background: #e74c3c; font-weight: bold; color: white; padding: 8px; border-radius: 5px;")
            self.lbl_pl_status.setText("Status: LISTENING (UDP 5005)")
            self.lbl_pl_status.setStyleSheet("color: #3498db; font-weight: bold;")
            
            self.pl_thread = PLReceiverThread(port=5005)
            self.pl_thread.data_received.connect(self.update_pl_ui)
            self.pl_thread.start()
            if hasattr(self, 'ai_log'):
                self.ai_log.append("<span style='color: #3498db;'>[PL] Activated Precision Landing listener...</span>")
        else:
            self.btn_pl_toggle.setText("ENABLE PRECISION LANDING")
            self.btn_pl_toggle.setStyleSheet("background: #2c3e50; font-weight: bold; color: white; padding: 8px; border-radius: 5px;")
            self.lbl_pl_status.setText("Status: STANDBY")
            self.lbl_pl_status.setStyleSheet("color: #f39c12; font-weight: bold;")
            self.lbl_pl_data.setText("X: 0.00 | Y: 0.00 | Dist: 0.00m")
            
            if self.pl_thread:
                self.pl_thread.stop()
                self.pl_thread = None
            
            if self.mav_worker and self.mav_worker.connection:
                self.mav_worker.connection.mav.rc_channels_override_send(
                    self.mav_worker.connection.target_system,
                    self.mav_worker.connection.target_component,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
            if hasattr(self, 'ai_log'):
                self.ai_log.append("<span style='color: #e74c3c;'>[PL] Deactivated Precision Landing.</span>")

    def update_pl_ui(self, data):
        is_tracking = data.get("tracking", False)
        if is_tracking:
            self.lbl_pl_status.setText("Status: TARGET LOCKED")
            self.lbl_pl_status.setStyleSheet("color: #2ecc71; font-weight: bold;")
            
            x, y, dist = data.get("x", 0.0), data.get("y", 0.0), data.get("dist", 0.0)
            self.lbl_pl_data.setText(f"X: {x:.2f} | Y: {y:.2f} | Dist: {dist:.2f}m")
            
            if self.mav_worker and self.mav_worker.connection:
                Kp = 60 
                rc_roll = 1500 + int(x * Kp)  
                rc_pitch = 1500 - int(y * Kp) 
                
                rc_roll = max(1420, min(1580, rc_roll))
                rc_pitch = max(1420, min(1580, rc_pitch))
                
                if abs(x) < 0.2 and abs(y) < 0.2 and dist > 0.5:
                    rc_throttle = 1450 
                elif dist <= 0.5 and dist > 0.0:
                    rc_throttle = 1300 
                else:
                    rc_throttle = 1500 
                
                self.mav_worker.connection.mav.rc_channels_override_send(
                    self.mav_worker.connection.target_system,
                    self.mav_worker.connection.target_component,
                    rc_roll, rc_pitch, rc_throttle, 
                    0, 0, 0, 0, 0
                )
        else:
            self.lbl_pl_status.setText("Status: TARGET LOST")
            self.lbl_pl_status.setStyleSheet("color: #e74c3c; font-weight: bold;")
            if self.mav_worker and self.mav_worker.connection:
                self.mav_worker.connection.mav.rc_channels_override_send(
                    self.mav_worker.connection.target_system,
                    self.mav_worker.connection.target_component,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
    # ======================================================================
    
    def request_all_params(self):
        if not self.mav_worker or not self.mav_worker.connection:
            QMessageBox.warning(self, "Error", "Cannot request parameters: No connection")
            return
            
        self.param_tree.clear()
        self.param_groups.clear()
        self.param_tree_items.clear()
        self.search_box.clear()
        
        self.mav_worker.connection.mav.param_request_list_send(
            self.mav_worker.connection.target_system,
            self.mav_worker.connection.target_component
        )
        self.append_log("<b style='color:#f39c12;'>[CONFIG] Loading parameter...</b>")

    def search_params(self, text):
        text = text.lower().strip()
        for i in range(self.param_tree.topLevelItemCount()):
            group = self.param_tree.topLevelItem(i)
            group_match = False
            
            for j in range(group.childCount()):
                child = group.child(j)
                if text in child.text(0).lower() or text in child.text(1).lower():
                    child.setHidden(False)
                    group_match = True
                else:
                    child.setHidden(True)
            
            group.setHidden(not group_match)
            if text and group_match:
                group.setExpanded(True)
            elif not text:
                group.setExpanded(False)

    def write_param(self, param_id, param_value):
        if not self.mav_worker or not self.mav_worker.connection:
            self.append_log("<span style='color:red;'>[CONFIG] Error message: Not connected</span>")
            return

        param_id_bytes = param_id.encode('utf-8')
        
        self.mav_worker.connection.mav.param_set_send(
            self.mav_worker.connection.target_system,
            self.mav_worker.connection.target_component,
            param_id_bytes,
            float(param_value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32 
        )
        self.append_log(f"<b style='color:#2ecc71;'>[CONFIG] Set command sent: {param_id} = {param_value}</b>")
        if hasattr(self, 'tts_worker'):
            self.tts_worker.speak(f"Parameter updated")

    def action_write_selected_param(self):
        selected = self.param_tree.selectedItems()
        if not selected:
            QMessageBox.warning(self, "Attention!", "Please click to select one parameter you wish to record!")
            return
            
        item = selected[0]
        pid = item.data(0, Qt.ItemDataRole.UserRole) 
        
        if not pid:
            QMessageBox.warning(self, "Attention!", "Please open the folder and select a specific parameter!")
            return
            
        try:
            new_val = float(item.text(1))
            self.write_param(pid, new_val)
            item.setBackground(1, QColor("#1abc9c")) 
            QTimer.singleShot(1000, lambda: item.setBackground(1, QColor(0,0,0,0)))
        except ValueError:
            QMessageBox.warning(self, "Error", "The value recorded must be a valid number!")

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
        def handle_connection_error(error_msg):
            self.btn_connect.setText("CONNECT")
            self.btn_connect.setStyleSheet("background-color: #27ae60; font-weight: bold; padding: 10px 20px; border-radius: 5px;")
            self.pfd.flight_mode = "OFFLINE"
            self.pfd.update()
            self.port_timer.start(1000)
            
            if self.mav_worker:
                self.mav_worker.deleteLater() 
                self.mav_worker = None
                self.page_mission.mav_worker = None

        if self.mav_worker and self.mav_worker.isRunning():
            self.mav_worker.stop()
            self.mav_worker = None
            self.page_mission.mav_worker = None
            
            self.btn_connect.setText("CONNECT")
            self.btn_connect.setStyleSheet("background-color: #27ae60; font-weight: bold; padding: 10px 20px; border-radius: 5px;")
            self.pfd.flight_mode = "OFFLINE"
            self.pfd.update()
            
            self.send_camera_command("STOP_STREAM")
            
            self.append_log("<b style='color: red;'>System Offline</b>")
            self.port_timer.start(1000)
            if hasattr(self, 'tts_worker'):
                self.tts_worker.speak("System Disconnected")
        else:
            baud = int(self.combo_baud.currentText()) 
            combo_val = self.combo_type.currentText()
            
            if "UDP 14550" in combo_val: 
                conn = 'udp:127.0.0.1:14550'  
            elif "TCP 5762" in combo_val: 
                conn = 'tcp:127.0.0.1:5762'   
            elif "UDP (Pi)" in combo_val: 
                conn = 'udp:0.0.0.0:14552'    
            elif "WIFI" in combo_val or "ESP8266" in combo_val: 
                conn = 'udp:0.0.0.0:14550' 
            else:
                full = self.combo_port.currentText()
                if not full: self.append_log("Select COM port!"); return
                conn = full.split()[0].strip() 
            # -------------------------------------------------------------

            self.port_timer.stop()
            self.append_log(f"Connecting to <b style='color: #3498db;'>{conn}</b>...")
            
            self.pfd.flight_mode = "WAITING..."
            self.pfd.update()
            
            self.mav_worker = MavlinkWorker(conn, baudrate=baud)
            self.page_mission.mav_worker = self.mav_worker
            self.page_setup.mav_worker = self.mav_worker
            self.mav_worker.data_changed.connect(self.update_ui)
            
            self.mav_worker.connection_failed.connect(handle_connection_error)
            self.mav_worker.link_status_changed.connect(self.handle_link_status)
            
            self.mav_worker.start()
            
            self.btn_connect.setText("DISCONNECT")
            self.btn_connect.setStyleSheet("background-color: #e74c3c; font-weight: bold; padding: 10px 20px; border-radius: 5px;")

            if hasattr(self, 'tts_worker'):
                self.tts_worker.speak("Connecting system")

    def update_map(self):
        lat, lon = self.drone_pos
        self.map_view.page().runJavaScript(f"updateDronePos({lat}, {lon}, {self.drone_heading}, {self.drone_course});")

    def update_ui(self, data):
        t = data.get("type")
        if not t: return
        
        # 1. RECORD LOG TELEMETRY & KML (Post-Flight)
        if self.is_recording and t == 'VFR_HUD':
            ts = QDateTime.currentDateTime().toString("hh:mm:ss.zzz")
            # Update CSV writer: HDOP/RSSI
            self.csv_writer.writerow([ts, self.pfd.flight_mode, self.pfd.is_armed, data.get('alt', 0), data.get('gs', 0), self.pfd.heading, self.drone_pos[0], self.drone_pos[1], self.pfd.batt_v, self.pfd.gps_hdop, self.pfd.rssi])
            
            # Record KML trajectory
            if hasattr(self, 'kml_file') and self.kml_file and self.drone_pos[0] != 0:
                self.kml_file.write(f"{self.drone_pos[1]},{self.drone_pos[0]},{data.get('alt', 0)} ")
        
        # 2. MAVLINK HANDLER
        if t == "LOG": 
            self.append_log(data.get("msg", ""))
            
        elif t == 'HEARTBEAT':
            current_mode = data.get('mode')
            current_armed = data.get('armed')
            
            if current_mode is not None and current_armed is not None:
                
                # --- Mode Handler ---
                if current_mode != self.last_mode:
                    self.append_log(f"Mode: <b style='color: white;'>{current_mode}</b>")
                    
                    if self.last_mode is not None and hasattr(self, 'tts_worker'): 
                        self.tts_worker.speak(f"Mode {current_mode}") 
                    
                    if self.last_mode is not None: 
                        self.pfd.status_msg = "Ready" 
                    self.last_mode = current_mode
                    
                # --- Armed/Disarmed ---
                if current_armed != self.last_armed:
                    is_armed_str = "ARMED" if current_armed else "DISARMED"
                    self.append_log(is_armed_str)
                    
                    if self.last_armed is not None and hasattr(self, 'tts_worker'): 
                        self.tts_worker.speak(f"Vehicle {is_armed_str}")
                    
                    if current_armed:
                        self.start_time = QDateTime.currentDateTime()
                        if self.drone_pos[0] != 0:
                            self.home_pos = self.drone_pos.copy()
                    if self.last_armed is not None: 
                        self.pfd.status_msg = "Ready"
                    self.last_armed = current_armed
                
                # --- Update HUD ---
                self.pfd.is_armed = current_armed
                self.pfd.flight_mode = current_mode
                self.pfd.update()
            
        elif t == 'ATTITUDE':
            self.pfd.roll, self.pfd.pitch = data.get("r", 0), data.get("p", 0)
            self.val_yaw.setText(f"{data.get('y', 0):.2f}")
            self.pfd.update()
            if hasattr(self, 'imu_graph'):
                self.imu_graph.update_attitude(data.get("r", 0), data.get("p", 0), data.get("y", 0))

            if hasattr(self, 'drone_3d'):
                self.drone_3d.update_attitude(data.get("r", 0), data.get("p", 0), data.get("y", 0))
            
            if hasattr(self, 'page_setup'):
                self.page_setup.update_attitude(data.get("r", 0), data.get("p", 0), data.get("y", 0))

        elif t == 'VIBRATION':
            if hasattr(self, 'imu_graph') and all(k in data for k in ("vib_x", "vib_y", "vib_z")):
                self.imu_graph.update_vibration(data["vib_x"], data["vib_y"], data["vib_z"])

        elif t == 'VFR_HUD':
            #self.pfd.alt = data.get('alt', 0)
            self.pfd.speed = data.get('gs', 0)
            self.pfd.heading = data.get('hdg', 0)
            #self.val_alt.setText(f"{self.pfd.alt:.1f}")
            self.val_gs.setText(f"{self.pfd.speed:.2f}")
            self.val_vs.setText(f"{data.get('vs', 0):.2f}")
            self.drone_heading = self.pfd.heading  
            self.update_map()
        elif t == 'GLOBAL_POSITION_INT':            #update alt
            self.pfd.alt = data.get('rel_alt', 0)
            self.val_alt.setText(f"{self.pfd.alt:.1f}")
            self.pfd.update()
        elif t == 'SYS_STATUS':
            self.pfd.batt_v = data.get('v', 0)
            self.pfd.batt_a = data.get('a', 0)
            self.pfd.batt_p = data.get('p', 0)
            self.pfd.ekf_status = "OK" if data.get('ekf', False) else "BAD"

            self.rc_healthy = data.get('rc_ok', True)
            if not self.rc_healthy:
                self.pfd.rssi = 0
            if 0 < self.pfd.batt_p < 20 and hasattr(self, 'tts_worker'):
                 self.tts_worker.speak("Warning, battery low")
                 
        elif t == 'STATUSTEXT':
            status_msg = data.get('status_msg', '')
            if "unknown message (44)" in status_msg:
                return
            self.append_log(f"-> {status_msg}")
            self.pfd.status_msg = status_msg
            self.pfd.update() 
            
            msg_lower = status_msg.lower()
            if ("fail" in msg_lower or "error" in msg_lower or "reject" in msg_lower) and hasattr(self, 'tts_worker'):
                self.tts_worker.speak(status_msg)
                
            QTimer.singleShot(5000, lambda: setattr(self.pfd, 'status_msg', 'Ready'))
            QTimer.singleShot(5005, self.pfd.update)
            
        elif t == 'GPS_RAW_INT':
            self.pfd.gps_sats = data.get('sats', 0)
            self.pfd.gps_hdop = data.get('hdop', 0.0) # HDOP
            self.pfd.gps_fix_type = data.get('fix_type', 0) #GPS FIX TYPE

            # --- GPS filter in home ---
            new_lat = data.get('lat', 16.0748)
            new_lon = data.get('lon', 108.1498)
            
            if new_lat != 0.0 and new_lon != 0.0:
                self.drone_pos = [new_lat, new_lon] 
            # --------------------------------------------
            
            self.drone_course = data.get('cog', 0)
            self.update_map()
            
            if hasattr(self, 'page_mission') and hasattr(self.page_mission, 'update_drone_position'):
                self.page_mission.update_drone_position(self.drone_pos[0], self.drone_pos[1], self.drone_heading, self.drone_course)
            
            # Calculate distance to home if armed and home position is set
            if self.home_pos is not None:
                dist = self.calc_distance(self.home_pos[0], self.home_pos[1], self.drone_pos[0], self.drone_pos[1])
                self.val_dist_mav.setText(f"{dist:.1f}") 
        
        elif t == 'GPS2_RAW':
                self.pfd.gps2_sats = data.get('sats2', 0)
                self.pfd.gps2_hdop = data.get('hdop2', 0.0)
                self.pfd.update()

        elif t == 'RADIO_STATUS':
            if getattr(self, 'rc_healthy', True):
                self.pfd.rssi = data.get('rssi', 0)
                
        elif t == 'RC_CHANNELS':
            if getattr(self, 'rc_healthy', True) and 'rssi' in data:
                self.pfd.rssi = data['rssi']
                
            if self.last_chans is not None and "chans" in data:
                for old, new in zip(self.last_chans, data["chans"]):
                    if 0 < old < 65000 and 0 < new < 65000 and abs(old - new) > 100:
                        if self.pfd.status_msg != "Ready": 
                            self.pfd.status_msg = "Ready"
                            self.pfd.update()
                        break 
            self.last_chans = data.get("chans", self.last_chans)
        
        elif t == 'NAV_CONTROLLER_OUTPUT':
            self.val_dist_wp.setText(f"{data.get('wp_dist', 0):.1f}")
            
        elif t == 'MISSION_CURRENT':
            self.current_wp = data.get('seq', 0)
            total_str = str(self.total_waypoints) if self.total_waypoints > 0 else "--"
            self.lbl_wp.setText(f"Waypoint: {self.current_wp} / {total_str}")
            
        elif t in ['MISSION_REQUEST', 'MISSION_ACK', 'MISSION_COUNT', 'MISSION_ITEM_INT']:
            if t == 'MISSION_COUNT':
                msg_obj = data.get('msg_obj')
                if msg_obj:
                    self.total_waypoints = msg_obj.count
                    self.lbl_wp.setText(f"Waypoint: {self.current_wp} / {self.total_waypoints}")
                    
            self.page_mission.handle_mission_msg(data)
        
        elif t == 'PARAM_VALUE':
            pid = data.get("param_id")
            pval = data.get("param_value")
            
            if pid == "WPNAV_SPEED":
                self.spin_wp_speed.setValue(pval / 100.0)
            
            if pid in self.param_tree_items:
                self.param_tree_items[pid].setText(1, str(pval))
            else:
                parts = pid.split('_', 1)
                group_name = parts[0] if len(parts) > 1 else "OTHER"
                
                if group_name not in self.param_groups:
                    group_item = QTreeWidgetItem([group_name, ""])
                    group_item.setBackground(0, QColor("#34495e")) 
                    group_item.setBackground(1, QColor("#34495e"))
                    font = group_item.font(0); font.setBold(True); group_item.setFont(0, font)
                    self.param_tree.addTopLevelItem(group_item)
                    self.param_groups[group_name] = group_item

                parent = self.param_groups[group_name]
                child_item = QTreeWidgetItem(parent, [pid, str(pval)])
                
                child_item.setFlags(child_item.flags() | Qt.ItemFlag.ItemIsEditable) 
                
                child_item.setData(0, Qt.ItemDataRole.UserRole, pid) 
                
                self.param_tree_items[pid] = child_item
    
    def calc_distance(self, lat1, lon1, lat2, lon2):
        """Calculate the distance (in meters) between two GPS coordinates using the Haversine formula"""
        R = 6371e3 
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

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
        if hasattr(self, 'pl_thread') and self.pl_thread:
            self.pl_thread.stop()
        event.accept()
    
    def confirm_and_send_cmd(self, action_name, color, cmd_func):
        dialog = SlideConfirmDialog(action_name, color, self)
        
        if dialog.exec() == QDialog.DialogCode.Accepted:
            cmd_func()
    
if __name__ == "__main__":
    try:
        import ctypes
        myappid = 'dut.mcs.pbl.1.0' 
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)
    except:
        pass

    app = QApplication(sys.argv)
    
    logo_path = CONFIG.get("paths", {}).get("app_logo", "logo.png")
    
    # --- 1. SET ICON ---
    from PyQt6.QtGui import QIcon, QPixmap
    from PyQt6.QtWidgets import QSplashScreen
    import os
    
    if os.path.exists(logo_path):
        app_icon = QIcon(logo_path) 
        app.setWindowIcon(app_icon)
        splash_pix = QPixmap(logo_path)
    else:
        app_icon = QIcon() 
        app.setWindowIcon(app_icon)
        splash_pix = QPixmap(500, 500)
        splash_pix.fill(Qt.GlobalColor.black)
        print(f"[WARNING] Logo file not found: {logo_path}")

    # --- 2. SPLASH SCREEN ---
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
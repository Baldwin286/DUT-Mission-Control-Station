import json
import os
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                             QLabel, QTableWidget, QTableWidgetItem, QHeaderView, 
                             QDoubleSpinBox, QAbstractItemView, QFrame)
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebEngineCore import QWebEnginePage, QWebEngineProfile
from PyQt6.QtCore import pyqtSignal, Qt
from pymavlink import mavutil

class MissionWebPage(QWebEnginePage):
    waypoint_clicked = pyqtSignal(float, float)
    fence_clicked = pyqtSignal(float, float)    #Geofence click signal
    fly_here_clicked = pyqtSignal(float, float) # <--- 1. Thêm signal bắt sự kiện Fly Here

    def acceptNavigationRequest(self, url, _type, isMainFrame):
        if url.scheme() == "wp":
            lat = float(url.host()); lon = float(url.path().strip('/'))
            self.waypoint_clicked.emit(lat, lon); return False 
        elif url.scheme() == "fence":
            lat = float(url.host()); lon = float(url.path().strip('/'))
            self.fence_clicked.emit(lat, lon); return False
        elif url.scheme() == "flyhere": # <--- 2. Thêm logic bắt URL từ giao diện Web
            lat = float(url.host()); lon = float(url.path().strip('/'))
            self.fly_here_clicked.emit(lat, lon); return False
        return super().acceptNavigationRequest(url, _type, isMainFrame)

class MissionPlannerApp(QWidget):
    def __init__(self, mav_worker=None, log_callback=None):
        super().__init__()
        self.mav_worker = mav_worker; self.log_callback = log_callback 
        self.waypoints = []; self.uploading = False; self.downloading = False
        self.init_ui(); self.load_map()

    def log(self, message):
        if self.log_callback: self.log_callback(message)

    def init_ui(self):
        layout = QHBoxLayout(self)
        left_panel = QFrame(); left_panel.setFixedWidth(380); left_panel.setStyleSheet("background-color: #1e1e1e; border-radius: 10px;")
        v_lay = QVBoxLayout(left_panel)
        v_lay.addWidget(QLabel("MISSION CONFIGURATION", styleSheet="font-size: 18px; font-weight: bold; color: #f39c12;"))

        h_alt = QHBoxLayout(); h_alt.addWidget(QLabel("Default Alt (m):"))
        self.spin_alt = QDoubleSpinBox(); self.spin_alt.setRange(2, 500); self.spin_alt.setValue(20)
        h_alt.addWidget(self.spin_alt); v_lay.addLayout(h_alt)

        self.table = QTableWidget(0, 4); self.table.setHorizontalHeaderLabels(["Seq", "Lat", "Lon", "Alt"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.setStyleSheet("QTableWidget { background: #1a1a1a; color: white; }")
        v_lay.addWidget(self.table)
        self.fence_points = []
        # FUNCTION BUTTONS
        self.btn_clear_ui = QPushButton("CLEAR UI MAP"); self.btn_clear_ui.clicked.connect(self.clear_mission)
        self.btn_read = QPushButton("READ FROM FC"); self.btn_read.clicked.connect(self.read_mission)
        self.btn_write = QPushButton("WRITE TO FC"); self.btn_write.clicked.connect(self.write_mission)
        self.btn_draw_fence = QPushButton("DRAW FENCE : OFF"); self.btn_draw_fence.setCheckable(True)
        self.btn_draw_fence.clicked.connect(self.toggle_fence_mode)
        self.btn_clear_fence = QPushButton("CLEAR FENCE"); self.btn_clear_fence.clicked.connect(self.clear_fence)
        self.btn_upload_fence = QPushButton("UPLOAD FENCE"); self.btn_upload_fence.clicked.connect(self.upload_fence)

        btn_style = "background: #2c3e50; color: white; font-weight: bold; padding: 10px; border-radius: 5px; margin-top: 2px;"
        
        # --- BUTTON 1: CLEAR IN FC CIRCUIT ---
        self.btn_delete_fc = QPushButton("DELETE FC MISSION"); self.btn_delete_fc.clicked.connect(self.delete_fc_mission)
        
        self.btn_auto = QPushButton("START AUTO MODE"); self.btn_auto.clicked.connect(self.set_auto_mode)
        
        # --- BUTTON 2: AUTO EXIT ---
        self.btn_abort = QPushButton("ABORT AUTO (LOITER)"); self.btn_abort.clicked.connect(self.abort_auto)
        
        btn_style = "background: #2c3e50; color: white; font-weight: bold; padding: 10px; border-radius: 5px; margin-top: 2px;"
        for b in [self.btn_draw_fence, self.btn_clear_fence, self.btn_upload_fence, self.btn_clear_ui, self.btn_read, self.btn_write, self.btn_delete_fc, self.btn_auto, self.btn_abort]:
            b.setStyleSheet(btn_style); v_lay.addWidget(b)
        
        self.btn_write.setStyleSheet(btn_style + "background: #27ae60;")
        self.btn_delete_fc.setStyleSheet(btn_style + "background: #e67e22;") 
        self.btn_auto.setStyleSheet(btn_style + "background: #8e44ad;")
        self.btn_abort.setStyleSheet(btn_style + "background: #c0392b;") 

        layout.addWidget(left_panel)
        self.map_view = QWebEngineView(); self.map_page = MissionWebPage(); self.map_page.waypoint_clicked.connect(self.add_wp)
        self.map_page.fence_clicked.connect(self.add_fence_point)
        
        # 3. Kết nối tín hiệu Fly Here với hàm thực thi MAVLink
        self.map_page.fly_here_clicked.connect(self.action_fly_here) 
        
        self.map_view.setPage(self.map_page); layout.addWidget(self.map_view)

    def load_map(self):
        # Xác định đường dẫn file map_mission.html nằm cùng thư mục
        base_dir = os.path.dirname(os.path.abspath(__file__))
        map_path = os.path.join(base_dir, "F:\PBL_DATN\Mavlink_app\map_data\map_mission.html")
        self.map_view.page().profile().setHttpUserAgent("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/122.0.0.0 Safari/537.36")
        try:
            with open(map_path, "r", encoding="utf-8") as f:
                html = f.read()
        except Exception as e:
            html = f"<h1>Lỗi tải bản đồ: {e}</h1>"
            self.log(f"<b style='color:red;'>Lỗi tải map_mission.html: {e}</b>")

        profile = QWebEngineProfile.defaultProfile()
        profile.setHttpUserAgent("Mozilla/5.0 Chrome/115.0 Safari/537.36")
        self.map_view.setHtml(html)

    def add_wp(self, lat, lon, alt=None):
        if alt is None: alt = self.spin_alt.value()
        seq = len(self.waypoints) + 1; self.waypoints.append({'lat': lat, 'lon': lon, 'alt': alt})
        row = self.table.rowCount(); self.table.insertRow(row)
        self.table.setItem(row, 0, QTableWidgetItem(str(seq))); self.table.setItem(row, 1, QTableWidgetItem(f"{lat:.6f}"))
        self.table.setItem(row, 2, QTableWidgetItem(f"{lon:.6f}")); self.table.setItem(row, 3, QTableWidgetItem(f"{alt}"))
        self.map_view.page().runJavaScript(f"addMarker({lat}, {lon}, {seq});")

    def clear_mission(self):
        self.waypoints = []; self.table.setRowCount(0); self.map_view.page().runJavaScript("clearMap();")

    def delete_fc_mission(self):
        if not self.mav_worker or not self.mav_worker.connection: return
        self.mav_worker.connection.mav.mission_clear_all_send(self.mav_worker.connection.target_system, self.mav_worker.connection.target_component)
        self.log("<b style='color:#e67e22;'>COMMAND: Mission Clear All sent to FC.</b>")

    def write_mission(self):
        if not self.mav_worker or not self.mav_worker.connection: return
        self.uploading = True
        self.mav_worker.connection.mav.mission_count_send(self.mav_worker.connection.target_system, self.mav_worker.connection.target_component, len(self.waypoints) + 1)
        self.log("<b style='color:#3498db;'>Uploading mission...</b>")

    def read_mission(self):
        if not self.mav_worker or not self.mav_worker.connection: return
        self.downloading = True; self.downloaded_wps = []
        self.mav_worker.connection.mav.mission_request_list_send(self.mav_worker.connection.target_system, self.mav_worker.connection.target_component)
        self.log("<b style='color:#f1c40f;'>Downloading mission from drone...</b>")

    def set_auto_mode(self):
        if not self.mav_worker: return
        self.mav_worker.connection.mav.set_mode_send(self.mav_worker.connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3) # Mode 3 = AUTO
        self.log("<b style='color:#9b59b6;'>Switching to AUTO Mode.</b>")

    def abort_auto(self):
        if not self.mav_worker: return
        self.mav_worker.connection.mav.set_mode_send(self.mav_worker.connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 5) # Mode 5 = LOITER
        self.log("<b style='color:#c0392b;'>ABORT AUTO: Switching to LOITER (Safe Break).</b>")

    def handle_mission_msg(self, data):
        if not self.mav_worker or not self.mav_worker.connection: return
        conn = self.mav_worker.connection; sys, comp = conn.target_system, conn.target_component
        t = data['type']; msg = data.get('msg_obj')
        if not msg: return

        if t in ['MISSION_REQUEST', 'MISSION_REQUEST_INT'] and self.uploading:
            if msg.seq == 0: conn.mav.mission_item_int_send(sys, comp, 0, 0, 16, 0, 1, 0, 0, 0, 0, 0, 0, 0)
            elif msg.seq <= len(self.waypoints):
                wp = self.waypoints[msg.seq-1]
                conn.mav.mission_item_int_send(sys, comp, msg.seq, 3, 16, 0, 1, 0, 0, 0, 0, int(wp['lat']*1e7), int(wp['lon']*1e7), wp['alt'])
        elif t == 'MISSION_COUNT' and self.downloading:
            self.download_count = msg.count
            if self.download_count > 0: conn.mav.mission_request_int_send(sys, comp, 0)
            else: self.downloading = False; self.log("Drone has no mission.")
        elif t == 'MISSION_ITEM_INT' and self.downloading:
            if msg.seq > 0: self.downloaded_wps.append({'lat': msg.x/1e7, 'lon': msg.y/1e7, 'alt': msg.z})
            if msg.seq < self.download_count - 1: conn.mav.mission_request_int_send(sys, comp, msg.seq + 1)
            else:
                self.downloading = False; conn.mav.mission_ack_send(sys, comp, 0)
                self.log(f"<b style='color:#2ecc71;'>DOWNLOAD SUCCESS!</b>"); self.clear_mission()
                for wp in self.downloaded_wps: self.add_wp(wp['lat'], wp['lon'], wp['alt'])
        elif t == 'MISSION_ACK':
            self.uploading = False; self.downloading = False
            if msg.type == 0: self.log("<b style='color:#2ecc71;'>MISSION ACCEPTED!</b>")
            
    def update_drone_position(self, lat, lon, heading, course=0):
        if hasattr(self, 'map_view'):
            js_code = f"if (typeof updateDronePos === 'function') {{ updateDronePos({float(lat)}, {float(lon)}, {float(heading)}, {float(course)}); }}"
            self.map_view.page().runJavaScript(js_code)

    # ================== XỬ LÝ GEOFENCE ==================
    def toggle_fence_mode(self, checked):
        if checked:
            self.btn_draw_fence.setText("DRAW FENCE : ON")
            self.btn_draw_fence.setStyleSheet("background: #e74c3c; color: white; font-weight: bold; padding: 10px; border-radius: 5px;")
            self.map_view.page().runJavaScript("toggleFenceMode(true);")
            self.log("<b style='color:#e74c3c;'>Fence Mode ON: Click map to draw boundary.</b>")
        else:
            self.btn_draw_fence.setText("DRAW FENCE : OFF")
            self.btn_draw_fence.setStyleSheet("background: #2c3e50; color: white; font-weight: bold; padding: 10px; border-radius: 5px;")
            self.map_view.page().runJavaScript("toggleFenceMode(false);")

    def add_fence_point(self, lat, lon):
        seq = len(self.fence_points) + 1
        self.fence_points.append({'lat': lat, 'lon': lon})
        self.map_view.page().runJavaScript(f"addFencePoint({lat}, {lon}, {seq});")

    def clear_fence(self):
        # 1. Xóa nét vẽ trên giao diện UI
        self.fence_points = []
        self.map_view.page().runJavaScript("clearFenceMap();")
        self.log("Geofence cleared from UI.")
        
        # 2. XÓA VÀ TẮT GEOFENCE DƯỚI MẠCH FC
        if self.mav_worker and self.mav_worker.connection:
            conn = self.mav_worker.connection
            
            # Tắt tính năng Geofence (FENCE_ENABLE = 0)
            conn.mav.param_set_send(
                conn.target_system, conn.target_component, 
                b'FENCE_ENABLE', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            
            # Đặt tổng số điểm về 0 để xóa sạch ranh giới cũ (FENCE_TOTAL = 0)
            conn.mav.param_set_send(
                conn.target_system, conn.target_component, 
                b'FENCE_TOTAL', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            
            self.log("<b style='color:#e67e22;'>[SYSTEM] Geofence DISABLED and CLEARED from FC memory!</b>")

    def configure_sitl_fence(self):
        if not self.mav_worker or not self.mav_worker.connection: 
            return
        
        conn = self.mav_worker.connection
        
        conn.mav.param_set_send(
            conn.target_system, conn.target_component, 
            b'FENCE_TYPE', 4, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        
        conn.mav.param_set_send(
            conn.target_system, conn.target_component, 
            b'FENCE_ACTION', 4, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        
        conn.mav.param_set_send(
            conn.target_system, conn.target_component, 
            b'FENCE_ENABLE', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        
        self.log("<b style='color:#3498db;'>[SYSTEM] SITL Fence Parameters Configured!</b>")

    def upload_fence(self):
        if not self.mav_worker or not self.mav_worker.connection: return
        count = len(self.fence_points)
        if count < 3:
            self.log("<b style='color:red;'>Error: Geofence requires at least 3 points!</b>")
            return

        conn = self.mav_worker.connection

        center_lat = sum(p['lat'] for p in self.fence_points) / count
        center_lon = sum(p['lon'] for p in self.fence_points) / count

        upload_points = [{'lat': center_lat, 'lon': center_lon}] 
        upload_points.extend(self.fence_points)                  
        upload_points.append(self.fence_points[0])               

        total_points = len(upload_points)

        conn.mav.param_set_send(conn.target_system, conn.target_component, b'FENCE_TOTAL', total_points, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        conn.mav.param_set_send(conn.target_system, conn.target_component, b'FENCE_TYPE', 4, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        conn.mav.param_set_send(conn.target_system, conn.target_component, b'FENCE_ACTION', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        for i, pt in enumerate(upload_points):
            conn.mav.fence_point_send(conn.target_system, conn.target_component, i, total_points, pt['lat'], pt['lon'])

        self.configure_sitl_fence()

        self.log(f"<b style='color:#2ecc71;'>UPLOADED GEOFENCE ({total_points} points incl. Return/Close) & ENABLED!</b>")

    # ================== XỬ LÝ FLY HERE (CLICK TO GO) ==================
    def action_fly_here(self, lat, lon):
        if not self.mav_worker or not self.mav_worker.connection:
            self.log("<b style='color:red;'>[ERROR] Connect to FC first to execute Fly Here!</b>")
            return
            
        alt = self.spin_alt.value()
        conn = self.mav_worker.connection
        
        # 1. Đổi chế độ sang GUIDED (Chế độ bắt buộc để nhận điểm tọa độ động)
        conn.mav.set_mode_send(
            conn.target_system, 
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 
            4 # Mode 4 trong ArduPilot là GUIDED
        )
        
        # 2. Gửi tọa độ tới (Sử dụng lệnh điều khiển quỹ đạo GLOBAL_RELATIVE_ALT)
        conn.mav.set_position_target_global_int_send(
            0, # time_boot_ms
            conn.target_system, 
            conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000, # Bitmask: Bỏ qua Vận tốc, Gia tốc, Lực quay; Chỉ lấy Tọa độ X,Y,Z
            int(lat * 1e7), 
            int(lon * 1e7), 
            alt,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        self.log(f"<b style='color:#e74c3c;'>[FLY HERE] Routing Drone to: {lat:.6f}, {lon:.6f} at {alt}m altitude.</b>")
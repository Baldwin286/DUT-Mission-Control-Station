import json
import os
import math 
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                             QLabel, QTableWidget, QTableWidgetItem, QHeaderView, 
                             QDoubleSpinBox, QAbstractItemView, QFrame, QComboBox)
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebEngineCore import QWebEnginePage, QWebEngineProfile
from PyQt6.QtCore import pyqtSignal, Qt
from pymavlink import mavutil

# ================== Ziczac path algorithm ==================
def generate_zigzag_path(polygon, spacing_m=5.0, direction=0):
    if len(polygon) < 3: return []

    LAT_TO_METER = 111320.0
    avg_lat = sum(p[0] for p in polygon) / len(polygon)
    LON_TO_METER = 111320.0 * math.cos(math.radians(avg_lat))

    ref_lat, ref_lon = polygon[0]
    poly_m = [((lon - ref_lon) * LON_TO_METER, (lat - ref_lat) * LAT_TO_METER) for lat, lon in polygon]

    max_len = 0
    best_angle = 0
    for i in range(len(poly_m)):
        p1 = poly_m[i]
        p2 = poly_m[(i + 1) % len(poly_m)]
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = math.hypot(dx, dy)
        if length > max_len:
            max_len = length
            best_angle = math.atan2(dy, dx)
            
    # Width direction = best_angle + 90 degrees
    if direction == 1:
        best_angle += math.pi / 2 

    cos_a = math.cos(-best_angle)
    sin_a = math.sin(-best_angle)
    rotated_poly = []
    for x, y in poly_m:
        rx = x * cos_a - y * sin_a
        ry = x * sin_a + y * cos_a
        rotated_poly.append((rx, ry))

    min_y = min(p[1] for p in rotated_poly)
    max_y = max(p[1] for p in rotated_poly)
    
    path_rotated = []
    sweep_y = min_y + spacing_m / 2.0
    left_to_right = True

    while sweep_y <= max_y:
        intersections = []
        for i in range(len(rotated_poly)):
            p1 = rotated_poly[i]
            p2 = rotated_poly[(i + 1) % len(rotated_poly)]
            if (p1[1] <= sweep_y <= p2[1]) or (p2[1] <= sweep_y <= p1[1]):
                if p1[1] != p2[1]: 
                    x_int = p1[0] + (sweep_y - p1[1]) * (p2[0] - p1[0]) / (p2[1] - p1[1])
                    intersections.append(x_int)

        intersections.sort()
        for i in range(0, len(intersections) - 1, 2):
            x1, x2 = intersections[i], intersections[i+1]
            if left_to_right: path_rotated.extend([(x1, sweep_y), (x2, sweep_y)])
            else: path_rotated.extend([(x2, sweep_y), (x1, sweep_y)])

        left_to_right = not left_to_right
        sweep_y += spacing_m

    cos_inv = math.cos(best_angle)
    sin_inv = math.sin(best_angle)
    
    path_ll = []
    for rx, ry in path_rotated:
        orig_x = rx * cos_inv - ry * sin_inv
        orig_y = rx * sin_inv + ry * cos_inv
        lat = ref_lat + (orig_y / LAT_TO_METER)
        lon = ref_lon + (orig_x / LON_TO_METER)
        path_ll.append((lat, lon))

    return path_ll
# ============================================================

class MissionWebPage(QWebEnginePage):
    waypoint_clicked = pyqtSignal(float, float)
    fence_clicked = pyqtSignal(float, float)
    fly_here_clicked = pyqtSignal(float, float)
    polygon_clicked = pyqtSignal(list) 

    def acceptNavigationRequest(self, url, _type, isMainFrame):
        if url.scheme() == "wp":
            lat = float(url.host()); lon = float(url.path().strip('/'))
            self.waypoint_clicked.emit(lat, lon); return False 
        elif url.scheme() == "fence":
            lat = float(url.host()); lon = float(url.path().strip('/'))
            self.fence_clicked.emit(lat, lon); return False
        elif url.scheme() == "flyhere":
            lat = float(url.host()); lon = float(url.path().strip('/'))
            self.fly_here_clicked.emit(lat, lon); return False
        # --- URL Polygon ---
        elif url.scheme() == "polygon":
            coords_str = url.path().strip('/') 
            points = []
            for pair in coords_str.split('_'): 
                if ',' in pair:
                    lat_str, lon_str = pair.split(',')
                    points.append((float(lat_str), float(lon_str)))
            self.polygon_clicked.emit(points); return False
        return super().acceptNavigationRequest(url, _type, isMainFrame)

class MissionPlannerApp(QWidget):
    mission_uploaded_signal = pyqtSignal(int)
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
        
        # --- 3. Direction between path ---
        h_space = QHBoxLayout(); h_space.addWidget(QLabel("Grid Spacing (m):"))
        self.spin_spacing = QDoubleSpinBox(); self.spin_spacing.setRange(1, 100); self.spin_spacing.setValue(5.0)
        h_space.addWidget(self.spin_spacing); v_lay.addLayout(h_space)

        h_dir = QHBoxLayout(); h_dir.addWidget(QLabel("Flight Direction:"))
        self.combo_dir = QComboBox()
        self.combo_dir.addItems(["Length", "Width"])
        self.combo_dir.setStyleSheet("background: #34495e; color: white;")
        h_dir.addWidget(self.combo_dir); v_lay.addLayout(h_dir)

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
        
        # --- 4. Button active ziczac ---
        self.btn_draw_poly = QPushButton("DRAW ZIG-ZAG AREA : OFF"); self.btn_draw_poly.setCheckable(True)
        self.btn_draw_poly.clicked.connect(self.toggle_poly_mode)
        
        self.btn_clear_fence = QPushButton("CLEAR FENCE"); self.btn_clear_fence.clicked.connect(self.clear_fence)
        self.btn_upload_fence = QPushButton("UPLOAD FENCE"); self.btn_upload_fence.clicked.connect(self.upload_fence)
        self.btn_delete_fc = QPushButton("DELETE FC MISSION"); self.btn_delete_fc.clicked.connect(self.delete_fc_mission)
        self.btn_auto = QPushButton("START AUTO MODE"); self.btn_auto.clicked.connect(self.set_auto_mode)
        self.btn_abort = QPushButton("ABORT AUTO (LOITER)"); self.btn_abort.clicked.connect(self.abort_auto)
        
        btn_style = "background: #2c3e50; color: white; font-weight: bold; padding: 10px; border-radius: 5px; margin-top: 2px;"
        
        btn_list = [self.btn_draw_fence, self.btn_draw_poly, self.btn_clear_fence, self.btn_upload_fence, 
                    self.btn_clear_ui, self.btn_read, self.btn_write, self.btn_delete_fc, self.btn_auto, self.btn_abort]
                    
        for b in btn_list:
            b.setStyleSheet(btn_style); v_lay.addWidget(b)
        
        self.btn_write.setStyleSheet(btn_style + "background: #27ae60;")
        self.btn_delete_fc.setStyleSheet(btn_style + "background: #e67e22;") 
        self.btn_auto.setStyleSheet(btn_style + "background: #8e44ad;")
        self.btn_abort.setStyleSheet(btn_style + "background: #c0392b;") 

        layout.addWidget(left_panel)
        self.map_view = QWebEngineView(); self.map_page = MissionWebPage(); self.map_page.waypoint_clicked.connect(self.add_wp)
        self.map_page.fence_clicked.connect(self.add_fence_point)
        self.map_page.fly_here_clicked.connect(self.action_fly_here) 
        
        # --- 5. Polygon handler ---
        self.map_page.polygon_clicked.connect(self.handle_polygon_received)
        
        self.map_view.setPage(self.map_page); layout.addWidget(self.map_view)

    def load_map(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        map_path = os.path.join(base_dir, "F:\PBL_DATN\Mavlink_app\map_data\map_mission.html")
        self.map_view.page().profile().setHttpUserAgent("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/122.0.0.0 Safari/537.36")
        try:
            with open(map_path, "r", encoding="utf-8") as f:
                html = f.read()
        except Exception as e:
            html = f"<h1>Error loading map: {e}</h1>"
            self.log(f"<b style='color:red;'>Error loading map_mission.html: {e}</b>")

        profile = QWebEngineProfile.defaultProfile()
        profile.setHttpUserAgent("Mozilla/5.0 Chrome/115.0 Safari/537.36")
        self.map_view.setHtml(html)

    def add_wp(self, lat, lon, alt=None):
        if alt is None: alt = self.spin_alt.value()
        seq = len(self.waypoints) + 1; self.waypoints.append({'lat': lat, 'lon': lon, 'alt': alt})
        row = self.table.rowCount(); self.table.insertRow(row)
        self.table.setItem(row, 0, QTableWidgetItem(str(seq))); self.table.setItem(row, 1, QTableWidgetItem(f"{lat:.6f}"))
        self.table.setItem(row, 2, QTableWidgetItem(f"{lon:.6f}")); self.table.setItem(row, 3, QTableWidgetItem(f"{alt}"))
        self.map_view.page().runJavaScript(f"if(typeof addMarker === 'function') addMarker({lat}, {lon}, {seq});")

    def clear_mission(self):
        self.waypoints = []; self.table.setRowCount(0); self.map_view.page().runJavaScript("if(typeof clearMap === 'function') clearMap();")

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
        self.mav_worker.connection.mav.set_mode_send(self.mav_worker.connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3) 
        self.log("<b style='color:#9b59b6;'>Switching to AUTO Mode.</b>")

    def abort_auto(self):
        if not self.mav_worker: return
        self.mav_worker.connection.mav.set_mode_send(self.mav_worker.connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 5) 
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
            if self.uploading and msg.type == 0:
                self.mission_uploaded_signal.emit(len(self.waypoints))
            self.uploading = False; self.downloading = False
            if msg.type == 0: self.log("<b style='color:#2ecc71;'>MISSION ACCEPTED!</b>")
            
    def update_drone_position(self, lat, lon, heading, course=0):
        if hasattr(self, 'map_view'):
            js_code = f"if (typeof updateDronePos === 'function') {{ updateDronePos({float(lat)}, {float(lon)}, {float(heading)}, {float(course)}); }}"
            self.map_view.page().runJavaScript(js_code)

    def toggle_fence_mode(self, checked):
        if checked:
            self.btn_draw_poly.setChecked(False); self.toggle_poly_mode(False)
            self.btn_draw_fence.setText("DRAW FENCE : ON")
            self.btn_draw_fence.setStyleSheet("background: #e74c3c; color: white; font-weight: bold; padding: 10px; border-radius: 5px;")
            self.map_view.page().runJavaScript("if(typeof toggleFenceMode === 'function') toggleFenceMode(true);")
            self.log("<b style='color:#e74c3c;'>Fence Mode ON: Click map to draw boundary.</b>")
        else:
            self.btn_draw_fence.setText("DRAW FENCE : OFF")
            self.btn_draw_fence.setStyleSheet("background: #2c3e50; color: white; font-weight: bold; padding: 10px; border-radius: 5px;")
            self.map_view.page().runJavaScript("if(typeof toggleFenceMode === 'function') toggleFenceMode(false);")

    # ================== ZICZAC POLYGON HANDLER ==================
    def toggle_poly_mode(self, checked):
        if checked:
            self.btn_draw_fence.setChecked(False); self.toggle_fence_mode(False)
            self.btn_draw_poly.setText("DRAW ZIG-ZAG AREA : ON")
            self.btn_draw_poly.setStyleSheet("background: #9b59b6; color: white; font-weight: bold; padding: 10px; border-radius: 5px;")
            self.map_view.page().runJavaScript("if(typeof togglePolyMode === 'function') togglePolyMode(true);")
            self.log("<b style='color:#9b59b6;'>Polygon Mode ON: Draw the area</b>")
        else:
            self.btn_draw_poly.setText("DRAW ZIG-ZAG AREA : OFF")
            self.btn_draw_poly.setStyleSheet("background: #2c3e50; color: white; font-weight: bold; padding: 10px; border-radius: 5px;")
            self.map_view.page().runJavaScript("if(typeof togglePolyMode === 'function') togglePolyMode(false);")

    def calc_distance(self, lat1, lon1, lat2, lon2):
        R = 6371e3 
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    # -----------------------------
    
    def handle_polygon_received(self, polygon_points):
        self.btn_draw_poly.setChecked(False); self.toggle_poly_mode(False)
        
        spacing = self.spin_spacing.value()
        direction = self.combo_dir.currentIndex()
        
        clean_poly = [polygon_points[0]]
        for p in polygon_points[1:]:
            dist = self.calc_distance(clean_poly[-1][0], clean_poly[-1][1], p[0], p[1])
            if dist > 1.0: 
                clean_poly.append(p)
        
        polygon_points = clean_poly 
        # --------------------------------------------------------------------------
        
        self.log(f"<b style='color:#3498db;'>[MISSION] Calculate the zigzag for the polygon {len(polygon_points)} vertices...</b>")
        
        zigzag_path = generate_zigzag_path(polygon_points, spacing, direction) 
        
        if not zigzag_path:
            self.log("<b style='color:red;'>[MISSION] Error: Cannot generate flight paths for this polygon.</b>")
            return
            
        self.log(f"<b style='color:#2ecc71;'>[MISSION] Successfully generated {len(zigzag_path)} Waypoints! Loading...</b>")
        
        alt = self.spin_alt.value()
        for lat, lon in zigzag_path:
            self.add_wp(lat, lon, alt)
    # ==========================================================

    def add_fence_point(self, lat, lon):
        seq = len(self.fence_points) + 1
        self.fence_points.append({'lat': lat, 'lon': lon})
        self.map_view.page().runJavaScript(f"if(typeof addFencePoint === 'function') addFencePoint({lat}, {lon}, {seq});")

    def clear_fence(self):
        self.fence_points = []
        self.map_view.page().runJavaScript("if(typeof clearFenceMap === 'function') clearFenceMap();")
        self.log("Geofence cleared from UI.")
        
        if self.mav_worker and self.mav_worker.connection:
            conn = self.mav_worker.connection
            conn.mav.param_set_send(conn.target_system, conn.target_component, b'FENCE_ENABLE', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            conn.mav.param_set_send(conn.target_system, conn.target_component, b'FENCE_TOTAL', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            self.log("<b style='color:#e67e22;'>[SYSTEM] Geofence DISABLED and CLEARED from FC memory!</b>")

    def configure_sitl_fence(self):
        if not self.mav_worker or not self.mav_worker.connection: return
        conn = self.mav_worker.connection
        conn.mav.param_set_send(conn.target_system, conn.target_component, b'FENCE_TYPE', 4, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        conn.mav.param_set_send(conn.target_system, conn.target_component, b'FENCE_ACTION', 4, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        conn.mav.param_set_send(conn.target_system, conn.target_component, b'FENCE_ENABLE', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        self.log("<b style='color:#3498db;'>[SYSTEM] SITL Fence Parameters Configured!</b>")

    def upload_fence(self):
        if not self.mav_worker or not self.mav_worker.connection: return
        count = len(self.fence_points)
        if count < 3:
            self.log("<b style='color:red;'>Error: Geofence requires at least 3 points!</b>"); return

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

    def action_fly_here(self, lat, lon):
        if not self.mav_worker or not self.mav_worker.connection:
            self.log("<b style='color:red;'>[ERROR] Connect to FC first to execute Fly Here!</b>")
            return
            
        alt = self.spin_alt.value()
        conn = self.mav_worker.connection
        conn.mav.set_mode_send(conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
        conn.mav.set_position_target_global_int_send(
            0, conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000, 
            int(lat * 1e7), int(lon * 1e7), alt,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        self.log(f"<b style='color:#e74c3c;'>[FLY HERE] Routing Drone to: {lat:.6f}, {lon:.6f} at {alt}m altitude.</b>")
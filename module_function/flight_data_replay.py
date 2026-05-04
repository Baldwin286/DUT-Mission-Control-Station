import csv
import os
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                             QLabel, QSlider, QFileDialog, QGroupBox, QFrame)
from PyQt6.QtCore import Qt, QTimer, QUrl
from PyQt6.QtWebEngineWidgets import QWebEngineView

class FlightReplayWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.replay_data = []
        self.current_index = 0
        self.is_playing = False
        
        # Timer run 10 frame/s
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.next_frame)
        
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 1. TOP CONTROL BAR
        ctrl_group = QGroupBox("REPLAY CONTROLS")
        ctrl_group.setStyleSheet("""
            QGroupBox { 
                font-weight: bold; color: #f1c40f; border: 1px solid #333; 
                border-radius: 5px; margin-top: 15px; 
            } 
            QGroupBox::title { 
                subcontrol-origin: margin; left: 10px; padding: 0 3px; 
            }
        """)
        
        ctrl_lay = QHBoxLayout(ctrl_group)
        
        self.btn_load = QPushButton("📂 LOAD CSV")
        self.btn_load.setStyleSheet("background: #2980b9; font-weight: bold; color: white; padding: 8px;")
        self.btn_load.clicked.connect(self.load_csv)
        
        self.btn_play = QPushButton("▶ PLAY")
        self.btn_play.setStyleSheet("background: #27ae60; font-weight: bold; color: white; padding: 8px;")
        self.btn_play.clicked.connect(self.toggle_play)
        
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setEnabled(False)
        self.slider.valueChanged.connect(self.slider_moved)
        
        self.lbl_time = QLabel("--:--:--")
        self.lbl_time.setStyleSheet("font-family: Consolas; font-weight:bold; font-size: 14px;")
        
        ctrl_lay.addWidget(self.btn_load)
        ctrl_lay.addWidget(self.btn_play)
        ctrl_lay.addWidget(self.slider)
        ctrl_lay.addWidget(self.lbl_time)
        layout.addWidget(ctrl_group)
        
        # 2. PARAM BOARD (HUD)
        hud_frame = QFrame()
        hud_frame.setStyleSheet("background: #1e1e1e; border-radius: 5px;")
        hud_lay = QHBoxLayout(hud_frame)
        
        self.lbl_mode = QLabel("MODE: ---")
        self.lbl_alt = QLabel("ALT: 0.0m")
        self.lbl_spd = QLabel("SPD: 0.0m/s")
        self.lbl_bat = QLabel("BAT: 0.0V")
        
        for lbl in [self.lbl_mode, self.lbl_alt, self.lbl_spd, self.lbl_bat]:
            lbl.setStyleSheet("color: #ecf0f1; font-weight: bold; font-size: 14px;")
            hud_lay.addWidget(lbl)
            
        layout.addWidget(hud_frame)
        
        # 3. SATTELLITE MAP VIEW
        self.map_view = QWebEngineView()
        self.map_view.setStyleSheet("border-radius: 10px; border: 2px solid #333;")
        
        # Allow local file access in QWebEngineView
        from PyQt6.QtWebEngineCore import QWebEngineSettings
        self.map_view.settings().setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
        self.map_view.settings().setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
        self.map_view.page().profile().setHttpUserAgent("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/123.0.0.0 Safari/537.36")
        
        # link to local map file 
        map_path = r"F:\PBL_DATN\DUT_MCS_App\map_data\map_mission.html"
        
        # Load map
        self.map_view.load(QUrl.fromLocalFile(map_path))
        layout.addWidget(self.map_view, 1)

    def load_csv(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open Telemetry CSV", "", "CSV Files (*.csv)")
        if not file_path: return
        
        self.replay_data = []
        try:
            with open(file_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    self.replay_data.append(row)
                    
            if self.replay_data:
                self.slider.setMaximum(len(self.replay_data) - 1)
                self.slider.setEnabled(True)
                self.slider.setValue(0)
                self.current_index = 0
                self.update_ui_frame()
                self.btn_play.setText("▶ PLAY")
                
        except Exception as e:
            print("Load CSV Error:", e)

    def toggle_play(self):
        if not self.replay_data: return
        
        self.is_playing = not self.is_playing
        if self.is_playing:
            self.btn_play.setText("⏸ PAUSE")
            self.btn_play.setStyleSheet("background: #e67e22; font-weight: bold; color: white; padding: 8px;")
            self.timer.start(100) 
        else:
            self.btn_play.setText("▶ PLAY")
            self.btn_play.setStyleSheet("background: #27ae60; font-weight: bold; color: white; padding: 8px;")
            self.timer.stop()

    def next_frame(self):
        if self.current_index < len(self.replay_data) - 1:
            self.current_index += 1
            self.slider.setValue(self.current_index) # Auto active update_ui_frame
        else:
            self.toggle_play() 

    def slider_moved(self, value):
        self.current_index = value
        self.update_ui_frame()

    def update_ui_frame(self):
        if not self.replay_data: return
        row = self.replay_data[self.current_index]
        
        self.lbl_time.setText(row.get('Timestamp', '--:--:--'))
        self.lbl_mode.setText(f"MODE: {row.get('Mode', '---')}")
        self.lbl_alt.setText(f"ALT: {float(row.get('Alt', 0)):.1f}m")
        self.lbl_spd.setText(f"SPD: {float(row.get('Spd', 0)):.1f}m/s")
        self.lbl_bat.setText(f"BAT: {float(row.get('Volt', 0)):.1f}V")
        
        # Update drone position on map
        lat = float(row.get('Lat', 0))
        lon = float(row.get('Lon', 0))
        yaw = float(row.get('Yaw', 0))
        
        if lat != 0 and lon != 0 and hasattr(self.map_view, 'page'):
            self.map_view.page().runJavaScript(f"if(typeof updateDronePos === 'function') updateDronePos({lat}, {lon}, {yaw}, {yaw});")
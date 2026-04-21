from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton
import pyqtgraph as pg

class IMUGraphWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.show_vibration = False
        
        self.r_b, self.p_b, self.y_b = [], [], []
        # Vibration array (AccX, AccY, AccZ)
        self.acc_x_b, self.acc_y_b, self.acc_z_b = [], [], []

        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Header and toggle button
        header_lay = QHBoxLayout()
        header_lay.addWidget(QLabel("REAL-TIME IMU DATA", styleSheet="font-weight:bold; color:white;"))
        
        self.btn_toggle = QPushButton("Switch to Vibration (Accel)")
        self.btn_toggle.setStyleSheet("background: #34495e; color: white; padding: 3px; border-radius: 3px;")
        self.btn_toggle.clicked.connect(self.toggle_mode)
        header_lay.addWidget(self.btn_toggle)
        layout.addLayout(header_lay)

        # graph
        self.graph = pg.PlotWidget()
        self.graph.setBackground('#121212')
        self.graph.setMaximumHeight(180)
        self.graph.setMinimumHeight(100)
        self.graph.showGrid(x=True, y=True, alpha=0.3)
        self.leg = self.graph.addLegend()
        self.leg.anchor((1,0),(1,0),offset=(-10,-16))
        
        self.c_r = self.graph.plot(pen='r', name="Roll/AccX")
        self.c_p = self.graph.plot(pen='g', name="Pitch/AccY")
        self.c_y = self.graph.plot(pen='b', name="Yaw/AccZ")
        layout.addWidget(self.graph)

    def toggle_mode(self):
        self.show_vibration = not self.show_vibration
        self.c_r.setData([])
        self.c_p.setData([])
        self.c_y.setData([])
        
        if self.show_vibration:
            self.btn_toggle.setText("Switch to Attitude (R/P/Y)")
            self.btn_toggle.setStyleSheet("background: #e67e22; color: white;")
            
            self.graph.enableAutoRange(axis='y')
            
            
        else:
            self.btn_toggle.setText("Switch to Vibration (Accel)")
            self.btn_toggle.setStyleSheet("background: #34495e; color: white;")
            self.graph.enableAutoRange(axis='y')

    def update_attitude(self, r, p, y):
        if not self.show_vibration:
            self.r_b.append(r)
            self.p_b.append(p)
            self.y_b.append(y)
            if len(self.r_b) > 100: 
                [b.pop(0) for b in [self.r_b, self.p_b, self.y_b]]
            self.c_r.setData(self.r_b)
            self.c_p.setData(self.p_b)
            self.c_y.setData(self.y_b)

    def update_vibration(self, vx, vy, vz):
        if not self.show_vibration: 
            return

        # Receive vibration from FC
        self.acc_x_b.append(vx)
        self.acc_y_b.append(vy)
        self.acc_z_b.append(vz)
        
        if len(self.acc_x_b) > 100: 
            [b.pop(0) for b in [self.acc_x_b, self.acc_y_b, self.acc_z_b]]
            
        self.c_r.setData(self.acc_x_b)
        self.c_p.setData(self.acc_y_b)
        self.c_y.setData(self.acc_z_b)
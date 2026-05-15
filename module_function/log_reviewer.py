import csv
import pyqtgraph as pg
import pyqtgraph.exporters
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                             QFileDialog, QListWidget, QListWidgetItem, QSplitter, 
                             QLabel, QMessageBox, QDoubleSpinBox, QGroupBox, QGridLayout)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor

class LogReviewWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.data_dict = {}
        self.time_array = []
        self.plot_lines = {}
        self.colors = ['#e74c3c', '#2ecc71', '#3498db', '#f1c40f', '#9b59b6', '#e67e22', '#1abc9c', '#ff7979', '#badc58']
        self.init_ui()

    def get_btn_style(self, bg_color, hover_color, border_color):
        return f"""
            QPushButton {{
                background-color: {bg_color};
                color: white; font-weight: bold; font-size: 13px; 
                padding: 10px; border-radius: 5px;
                border: 2px solid {border_color};
                border-bottom: 3px solid {border_color}; 
            }}
            QPushButton:hover {{
                background-color: {hover_color};
                border: 2px solid #ecf0f1; 
                border-bottom: 3px solid #bdc3c7;
            }}
            QPushButton:pressed {{
                background-color: {border_color}; 
                border: 2px solid {bg_color};
                padding-top: 12px; padding-bottom: 8px;
            }}
        """

    def init_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        left_panel = QVBoxLayout()
        
        self.btn_load = QPushButton("LOAD CSV LOG")
        self.btn_load.setStyleSheet(self.get_btn_style("#d35400", "#e67e22", "#a04000")) 
        self.btn_load.clicked.connect(self.load_csv)

        self.btn_clear = QPushButton("CLEAR GRAPH")
        self.btn_clear.setStyleSheet(self.get_btn_style("#7f8c8d", "#95a5a6", "#2c3e50")) 
        self.btn_clear.clicked.connect(self.clear_graph)

        self.btn_export = QPushButton("EXPORT GRAPH")
        self.btn_export.setStyleSheet(self.get_btn_style("#27ae60", "#2ecc71", "#145a32")) 
        self.btn_export.clicked.connect(self.export_matlab_graph)

        y_group = QGroupBox("Y-AXIS LIMITER")
        y_group.setStyleSheet("QGroupBox { font-weight: bold; color: #f1c40f; border: 1px solid #333; margin-top: 10px; padding-top: 15px; }")
        y_lay = QGridLayout(y_group)
        
        self.chk_auto_y = QPushButton("AUTO RANGE: ON")
        self.chk_auto_y.setCheckable(True)
        self.chk_auto_y.setChecked(True)
        self.chk_auto_y.setStyleSheet(self.get_btn_style("#2980b9", "#3498db", "#1a5276"))
        self.chk_auto_y.clicked.connect(self.toggle_auto_y)
        
        spin_style = """
            QDoubleSpinBox {
                background: #2c3e50; 
                color: white; 
                border: 1px solid #34495e; 
                border-radius: 3px;
                padding: 0px 15px 0px 5px; 
                min-height: 26px; 
            }
        """
        
        self.spin_y_max = QDoubleSpinBox()
        self.spin_y_max.setRange(-10000, 10000)
        self.spin_y_max.setSingleStep(5.0)
        self.spin_y_max.setValue(10.0)
        self.spin_y_max.setStyleSheet(spin_style)
        self.spin_y_max.setEnabled(False)
        self.spin_y_max.valueChanged.connect(self.manual_y_change)
        
        self.spin_y_min = QDoubleSpinBox()
        self.spin_y_min.setRange(-10000, 10000)
        self.spin_y_min.setSingleStep(5.0)
        self.spin_y_min.setValue(-10.0)
        self.spin_y_min.setStyleSheet(spin_style)
        self.spin_y_min.setEnabled(False)
        self.spin_y_min.valueChanged.connect(self.manual_y_change)

        y_lay.addWidget(self.chk_auto_y, 0, 0, 1, 2)
        y_lay.addWidget(QLabel("Max Y:", styleSheet="color: white;"), 1, 0)
        y_lay.addWidget(self.spin_y_max, 1, 1)
        y_lay.addWidget(QLabel("Min Y:", styleSheet="color: white;"), 2, 0)
        y_lay.addWidget(self.spin_y_min, 2, 1)

        self.list_vars = QListWidget()
        self.list_vars.setStyleSheet("""
            QListWidget { background: #1e1e1e; color: #ecf0f1; border: 1px solid #333; font-family: Consolas; }
            QListWidget::item { padding: 5px; }
            QListWidget::item:hover { background: #2c3e50; }
            QListWidget::item:selected { background: #34495e; color: #f1c40f; font-weight: bold; }
        """)
        self.list_vars.itemChanged.connect(self.on_item_changed)

        left_panel.addWidget(self.btn_load)
        left_panel.addWidget(self.btn_clear)
        left_panel.addWidget(self.btn_export) 
        left_panel.addWidget(y_group)
        left_panel.addWidget(QLabel("Available Parameters:", styleSheet="color: #f39c12; font-weight: bold; margin-top: 5px;"))
        left_panel.addWidget(self.list_vars)

        left_widget = QWidget()
        left_widget.setLayout(left_panel)
        left_widget.setMaximumWidth(280)

        right_panel = QVBoxLayout()
        right_panel.setContentsMargins(0,0,0,0)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#121212')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.addLegend(offset=(-10, 10))
        self.plot_widget.setLabel('bottom', 'Time (Seconds)')
        
        self.plot_widget.setMouseEnabled(x=True, y=True)
        self.plot_widget.getViewBox().setAutoVisible(y=True) 
        
        self.vLine = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen(color='w', width=1, style=Qt.PenStyle.DashLine))
        self.snap_points = pg.ScatterPlotItem(size=8, pen=pg.mkPen(None)) 
        
        self.tooltip = pg.TextItem(text="", color="w", fill=(0, 0, 0, 200))
        self.tooltip.setZValue(1000) 

        self.plot_widget.addItem(self.vLine, ignoreBounds=True)
        self.plot_widget.addItem(self.snap_points)
        self.plot_widget.addItem(self.tooltip, ignoreBounds=True)
        
        self.vLine.hide()
        self.tooltip.hide() 

        self.proxy = pg.SignalProxy(self.plot_widget.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved)

        right_panel.addWidget(self.plot_widget)
        right_widget = QWidget()
        right_widget.setLayout(right_panel)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setSizes([250, 900])

        layout.addWidget(splitter)

    def toggle_auto_y(self, checked):
        if checked:
            self.chk_auto_y.setText("AUTO RANGE: ON")
            self.chk_auto_y.setStyleSheet(self.get_btn_style("#2980b9", "#3498db", "#1a5276"))
            self.spin_y_max.setEnabled(False)
            self.spin_y_min.setEnabled(False)
            self.plot_widget.getViewBox().enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)
            self.plot_widget.getViewBox().autoRange() 
        else:
            self.chk_auto_y.setText("MANUAL OVERRIDE")
            self.chk_auto_y.setStyleSheet(self.get_btn_style("#c0392b", "#e74c3c", "#922b21"))
            self.spin_y_max.setEnabled(True)
            self.spin_y_min.setEnabled(True)
            self.plot_widget.getViewBox().enableAutoRange(axis=pg.ViewBox.YAxis, enable=False)
            self.manual_y_change()

    def manual_y_change(self):
        if not self.chk_auto_y.isChecked():
            ymin = self.spin_y_min.value()
            ymax = self.spin_y_max.value()
            if ymax > ymin:
                self.plot_widget.setYRange(ymin, ymax, padding=0)

    def load_csv(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open Telemetry or IMU CSV", "", "CSV Files (*.csv)")
        if not path: return

        self.clear_graph()
        self.list_vars.clear()
        self.data_dict.clear()

        try:
            with open(path, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                headers = next(reader)
                
                for h in headers:
                    self.data_dict[h] = []

                for row in reader:
                    for i, val in enumerate(row):
                        if i < len(headers):
                            self.data_dict[headers[i]].append(val)

            time_col = headers[0]
            self.time_array = []
            start_time = None
            
            for t_str in self.data_dict[time_col]:
                try:
                    h, m, s_ms = t_str.split(':')
                    s, ms = s_ms.split('.')
                    t_sec = int(h)*3600 + int(m)*60 + int(s) + int(ms)/1000.0
                    
                    if start_time is None: start_time = t_sec
                    self.time_array.append(t_sec - start_time) 
                except:
                    self.time_array.append(len(self.time_array) * 0.1) 

            for h in headers[1:]:
                numeric_data = []
                is_numeric = False
                
                for val in self.data_dict[h]:
                    try:
                        numeric_data.append(float(val))
                        is_numeric = True
                    except ValueError:
                        numeric_data.append(0.0)
                
                if is_numeric:
                    self.data_dict[h] = numeric_data 
                    
                    item = QListWidgetItem(f" {h}")
                    item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                    item.setCheckState(Qt.CheckState.Unchecked)
                    self.list_vars.addItem(item)
                    
            QMessageBox.information(self, "Success", f"Successfully loaded the {len(self.time_array)} data sample!")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to parse CSV:\n{str(e)}")

    def on_item_changed(self, item):
        var_name = item.text().strip()
        
        if item.checkState() == Qt.CheckState.Checked:
            if var_name not in self.plot_lines:
                color = QColor(self.colors[len(self.plot_lines) % len(self.colors)])
                pen = pg.mkPen(color=color, width=2)
                line = self.plot_widget.plot(self.time_array, self.data_dict[var_name], name=var_name, pen=pen)
                self.plot_lines[var_name] = line
        else:
            if var_name in self.plot_lines:
                self.plot_widget.removeItem(self.plot_lines[var_name])
                del self.plot_lines[var_name]

    def clear_graph(self):
        for i in range(self.list_vars.count()):
            self.list_vars.item(i).setCheckState(Qt.CheckState.Unchecked)
        self.plot_widget.clear()
        self.plot_lines.clear()
        
        self.plot_widget.addItem(self.vLine, ignoreBounds=True)
        self.plot_widget.addItem(self.snap_points)
        self.plot_widget.addItem(self.tooltip, ignoreBounds=True)
        self.vLine.hide()
        self.tooltip.hide()

    def mouseMoved(self, evt):
        pos = evt[0]
        vb = self.plot_widget.plotItem.vb
        
        if vb.sceneBoundingRect().contains(pos):
            mousePoint = vb.mapSceneToView(pos)
            x_mouse = mousePoint.x()
            
            if not self.time_array or not self.plot_lines: 
                self.tooltip.hide()
                self.vLine.hide()
                self.snap_points.setData([])
                return
            
            import bisect
            idx = bisect.bisect_left(self.time_array, x_mouse)
            if idx >= len(self.time_array): idx = len(self.time_array) - 1
            elif idx > 0 and abs(x_mouse - self.time_array[idx-1]) < abs(x_mouse - self.time_array[idx]):
                idx = idx - 1
                
            snapped_x = self.time_array[idx]
            
            self.vLine.setPos(snapped_x)
            self.vLine.show()
            
            info_text = f"<span style='color: #f1c40f; font-size: 11pt;'>Time: {snapped_x:.2f}s</span><br>"
            spots = []
            
            for var_name in self.plot_lines.keys():
                y_val = self.data_dict[var_name][idx]
                color_hex = self.plot_lines[var_name].opts['pen'].color().name()
                
                info_text += f"<b style='color: {color_hex};'>{var_name}:</b> <span style='color: white;'>{y_val:.2f}</span><br>"
                spots.append({'pos': (snapped_x, y_val), 'brush': QColor(color_hex)})
                
            self.snap_points.setData(spots)
            
            self.tooltip.setHtml(info_text)
            self.tooltip.setPos(snapped_x, mousePoint.y())
            self.tooltip.setAnchor((-0.1, 1.2))
            self.tooltip.show()
        else:
            self.tooltip.hide()
            self.vLine.hide()
            self.snap_points.setData([])
            
    def export_matlab_graph(self):
        if not self.plot_lines:
            QMessageBox.warning(self, "Warning", "No graphs to export!")
            return
            
        from PyQt6.QtWidgets import QInputDialog, QApplication
        from PyQt6.QtGui import QFont
        
        title_text, ok = QInputDialog.getText(self, "Graph name", "Enter a chart name:")
        if not ok: return

        save_path, _ = QFileDialog.getSaveFileName(self, "Save Graph as PNG", "DATN_Graph.png", "PNG Image (*.png)")
        if not save_path: return

        try:
            self.vLine.hide()
            self.tooltip.hide()
            self.snap_points.setData([])

            old_pens = {}

            self.plot_widget.setBackground('w')
            
            self.plot_widget.plotItem.layout.setContentsMargins(30, 20, 40, 30) 
            
            if title_text.strip():
                self.plot_widget.setTitle(title_text.strip(), color='k', size='22pt')
                self.plot_widget.plotItem.titleLabel.setContentsMargins(0, 0, 0, 20)

            view_box = self.plot_widget.getViewBox()
            old_x_range = view_box.viewRange()[0]
            old_y_range = view_box.viewRange()[1]
            
            view_box.autoRange(padding=0.02)

            
            axis_pen = pg.mkPen(color='k', width=1.2)
            label_style = {'color': '#000', 'font-size': '14pt'} 
            
            for axis in ['bottom', 'left', 'top', 'right']:
                ax = self.plot_widget.getAxis(axis)
                ax.setPen(axis_pen)
                ax.setTextPen('k')
                
                font = QFont()
                font.setPointSize(12) 
                ax.setTickFont(font)
                
                if axis == 'bottom':
                    ax.setLabel('Time (Seconds)', **label_style)
                elif axis == 'left':
                    ax.setLabel('Amplitude', **label_style)

            for name, line in self.plot_lines.items():
                old_pens[name] = line.opts['pen']
                color = old_pens[name].color()
                line.setPen(pg.mkPen(color=color, width=2.5)) 

            legend = self.plot_widget.plotItem.legend
            old_legend_texts = {}
            if legend:
                for item in legend.items:
                    label = item[1]
                    old_legend_texts[label] = label.text
                    raw_text = label.text.replace('<span style="color: white;">', '').replace('</span>', '')
                    label.setText(f'<span style="color: black; font-size: 14pt;">{raw_text}</span>')

            self.plot_widget.showGrid(x=True, y=True, alpha=0.2)
            
            QApplication.processEvents() 

            exporter = pg.exporters.ImageExporter(self.plot_widget.plotItem)
            exporter.parameters()['width'] = 1920 
            exporter.export(save_path)

            self.plot_widget.setBackground('#121212')
            self.plot_widget.setTitle(None) 
            self.plot_widget.plotItem.layout.setContentsMargins(0, 0, 0, 0) 
            self.plot_widget.plotItem.titleLabel.setContentsMargins(0, 0, 0, 0)
            
            view_box.setRange(xRange=old_x_range, yRange=old_y_range, padding=0)

            dark_pen = pg.mkPen(color='#888888', width=1)
            dark_label_style = {'color': '#888888', 'font-size': '11pt'}
            
            for axis in ['bottom', 'left', 'top', 'right']:
                ax = self.plot_widget.getAxis(axis)
                ax.setPen(dark_pen)
                ax.setTextPen('#888888')
                
                font = QFont()
                font.setPointSize(10)
                ax.setTickFont(font)
                
                if axis == 'bottom':
                    ax.setLabel('Time (Seconds)', **dark_label_style)
                elif axis == 'left':
                    ax.setLabel('', **dark_label_style)

            for name, line in self.plot_lines.items():
                line.setPen(old_pens[name])

            if legend:
                for label, old_text in old_legend_texts.items():
                    label.setText(old_text)

            self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
            
            QMessageBox.information(self, "Success", f"The graph has been saved at:\n{save_path}")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error when exporting image: {e}")
            self.plot_widget.setBackground('#121212')
            self.plot_widget.plotItem.layout.setContentsMargins(0, 0, 0, 0)
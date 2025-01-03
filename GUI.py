import sys
import os
from PyQt6.QtWidgets import (QApplication, QMainWindow, QPushButton, QFileDialog,
                           QVBoxLayout, QHBoxLayout, QWidget, QLabel, QTableWidget,
                           QTableWidgetItem, QStyle, QStatusBar, QMessageBox, QLineEdit, QGridLayout)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QMovie
import numpy as np
import opensim
from load_file import load_mot

class OpenGRFGui(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenGRF File Loader")
        self.setMinimumSize(1000, 700)
        
        # Initialize variables
        self.init_variables()
        
        # Set the style
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e1e;
            }
            QPushButton {
                background-color: #2d2d2d;
                color: #e0e0e0;
                border: 1px solid #3d3d3d;
                padding: 8px 16px;
                border-radius: 4px;
                font-size: 13px;
                min-width: 150px;
                max-width: 200px;
            }
            QPushButton:hover {
                background-color: #3d3d3d;
                border: 1px solid #4d4d4d;
            }
            QPushButton:disabled {
                background-color: #252525;
                color: #666666;
                border: 1px solid #2d2d2d;
            }
            QLabel {
                color: #e0e0e0;
                font-size: 13px;
            }
            QTableWidget {
                background-color: #2d2d2d;
                color: #e0e0e0;
                gridline-color: #3d3d3d;
                border: none;
            }
            QHeaderView::section {
                background-color: #252525;
                color: #e0e0e0;
                padding: 5px;
                border: 1px solid #3d3d3d;
            }
            QStatusBar {
                color: #e0e0e0;
            }
            QLineEdit {
                background-color: #2d2d2d;
                color: #e0e0e0;
                border: 1px solid #3d3d3d;
                padding: 5px;
                border-radius: 4px;
                font-size: 13px;
            }
            QLineEdit:focus {
                border: 1px solid #4d4d4d;
                background-color: #333333;
            }
            QLineEdit:disabled {
                background-color: #252525;
                color: #666666;
            }
        """)

        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Model section
        model_section = QVBoxLayout()
        model_label = QLabel("1. Load OpenSim Model (.osim)")
        self.model_status = QLabel("No model loaded")
        self.model_status.setStyleSheet("color: #ffa000;")
        self.load_model_btn = QPushButton("📂 Select Model")
        self.load_model_btn.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_DialogOpenButton))
        self.load_model_btn.clicked.connect(self.load_model)
        
        model_section.addWidget(model_label)
        model_section.addWidget(self.model_status)
        model_section.addWidget(self.load_model_btn)
        layout.addLayout(model_section)
        
        # Spacer
        layout.addSpacing(20)
        
        # Motion section
        motion_section = QVBoxLayout()
        motion_label = QLabel("2. Load Motion File (.mot)")
        self.motion_status = QLabel("No motion data loaded")
        self.motion_status.setStyleSheet("color: #ffa000;")
        self.load_motion_btn = QPushButton("📂 Select Motion")
        self.load_motion_btn.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_DialogOpenButton))
        self.load_motion_btn.clicked.connect(self.load_motion)
        self.load_motion_btn.setEnabled(False)
        
        motion_section.addWidget(motion_label)
        motion_section.addWidget(self.motion_status)
        motion_section.addWidget(self.load_motion_btn)
        layout.addLayout(motion_section)
        
        # Spacer
        layout.addSpacing(20)
        
        # Calculate GRF section
        grf_section = QVBoxLayout()
        grf_label = QLabel("3. Calculate Ground Reaction Force")
        self.grf_status = QLabel("Ready to calculate")
        self.grf_status.setStyleSheet("color: #ffa000;")
        
        # Parameters layout
        params_layout = QGridLayout()
        params_layout.setColumnMinimumWidth(1, 100)  # Set width for input columns
        params_layout.setColumnMinimumWidth(3, 100)
        params_layout.setHorizontalSpacing(10)  # Add some spacing between columns
        params_layout.setAlignment(Qt.AlignmentFlag.AlignLeft)  # Align entire layout to left
        
        # Create a container widget for parameters
        params_container = QWidget()
        params_container.setLayout(params_layout)
        
        # Time range
        time_label1 = QLabel("Start time:")
        time_label1.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        time_label2 = QLabel("End time:")
        time_label2.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        
        self.start_time = QLineEdit()
        self.start_time.setFixedWidth(100)
        self.start_time.textChanged.connect(self.update_table)
        self.end_time = QLineEdit()
        self.end_time.setFixedWidth(100)
        self.end_time.textChanged.connect(self.update_table)
        
        params_layout.addWidget(time_label1, 0, 0)
        params_layout.addWidget(self.start_time, 0, 1)
        params_layout.addWidget(time_label2, 0, 2)
        params_layout.addWidget(self.end_time, 0, 3)
        
        # Cut-off frequency
        freq_label = QLabel("Low pass cut-off frequency (Hz):")
        freq_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.cutoff_freq = QLineEdit("8")
        self.cutoff_freq.setFixedWidth(100)
        params_layout.addWidget(freq_label, 1, 0)
        params_layout.addWidget(self.cutoff_freq, 1, 1)
        
        # Force threshold
        force_label = QLabel("Spheres force threshold (N):")
        force_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.force_threshold = QLineEdit("400")
        self.force_threshold.setFixedWidth(100)
        params_layout.addWidget(force_label, 1, 2)
        params_layout.addWidget(self.force_threshold, 1, 3)
        
        # Add parameters to GRF section with left alignment
        param_container_layout = QHBoxLayout()
        param_container_layout.addWidget(params_container)
        
        # Add GIF to the right side
        gif_label = QLabel()
        gif_label.setFixedSize(100, 100)  # 크기 조절 가능
        self.movie = QMovie("walking.gif")  # GIF 파일 경로 지정
        self.movie.setScaledSize(gif_label.size())
        gif_label.setMovie(self.movie)
        self.movie.start()
        
        param_container_layout.addWidget(gif_label)
        param_container_layout.addStretch()
        
        grf_section.addWidget(grf_label)
        grf_section.addWidget(self.grf_status)
        grf_section.addLayout(param_container_layout)
        
        # Buttons layout
        buttons_layout = QHBoxLayout()
        
        # Calculate button
        self.calculate_btn = QPushButton("➡️ Calculate GRF")
        self.calculate_btn.clicked.connect(self.calculate_grf)
        self.calculate_btn.setEnabled(False)
        buttons_layout.addWidget(self.calculate_btn)
        
        # New Motion button
        self.new_motion_btn = QPushButton("🔄 New Motion")
        self.new_motion_btn.clicked.connect(self.reset_all)
        self.new_motion_btn.setEnabled(False)
        buttons_layout.addWidget(self.new_motion_btn)
        
        # Add stretch to push buttons to the left
        buttons_layout.addStretch()
        
        grf_section.addLayout(buttons_layout)
        
        layout.addLayout(grf_section)
        
        # Data preview
        preview_label = QLabel("Motion Data Preview:")
        layout.addWidget(preview_label)
        
        self.table = QTableWidget()
        self.table.setAlternatingRowColors(True)
        layout.addWidget(self.table)
        
        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        
    def init_variables(self):
        """Initialize/Reset all variables to their default states"""
        self.model = None
        self.motion_data = None
        self.motion_headers = None
        self.model_path = None
        self.motion_path = None
        self.calculation_done = False
    
    def reset_all(self):
        """Reset the application to its initial state"""
        # Reset variables
        self.init_variables()
        
        # Reset UI elements
        self.model_status.setText("No model loaded")
        self.model_status.setStyleSheet("color: #ffa000;")
        self.motion_status.setText("No motion data loaded")
        self.motion_status.setStyleSheet("color: #ffa000;")
        self.grf_status.setText("Ready to calculate")
        self.grf_status.setStyleSheet("color: #ffa000;")
        
        # Reset buttons
        self.load_motion_btn.setEnabled(False)
        self.calculate_btn.setEnabled(False)
        self.new_motion_btn.setEnabled(False)
        
        # Clear input fields
        self.start_time.setText("")
        self.end_time.setText("")
        self.cutoff_freq.setText("8")
        self.force_threshold.setText("400")
        
        # Clear table
        self.table.setRowCount(0)
        self.table.setColumnCount(0)
        
        # Update status
        self.status_bar.showMessage("Ready for new motion analysis ")
    
    def load_model(self):
        try:
            file_name, _ = QFileDialog.getOpenFileName(
                self,
                "Select OpenSim Model",
                "",
                "OpenSim Model (*.osim);;All Files (*.*)"
            )
            
            if file_name:
                # Load the model using OpenSim API
                self.model = opensim.Model(file_name)
                self.model_path = file_name
                
                # Update UI
                self.model_status.setText(f" {os.path.basename(file_name)}")
                self.model_status.setStyleSheet("color: #66bb6a;")
                self.load_motion_btn.setEnabled(True)
                self.new_motion_btn.setEnabled(True)
                self.status_bar.showMessage("Model loaded successfully ")
                
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error loading model: {str(e)}")
            self.status_bar.showMessage("Error loading model ")
    
    def load_motion(self):
        try:
            file_name, _ = QFileDialog.getOpenFileName(
                self,
                "Select Motion File",
                os.path.dirname(self.model_path) if self.model_path else "",
                "Motion File (*.mot);;All Files (*.*)"
            )
            
            if file_name:
                # Load motion data using the load_mot function
                self.motion_data, self.motion_headers = load_mot(file_name)
                self.motion_path = file_name
                
                # Update UI
                self.motion_status.setText(f" {os.path.basename(file_name)}")
                self.motion_status.setStyleSheet("color: #66bb6a;")
                self.status_bar.showMessage("Motion data loaded successfully ")
                
                # Enable calculate button and set default times
                self.calculate_btn.setEnabled(True)
                self.new_motion_btn.setEnabled(True)
                if len(self.motion_data) > 0:
                    self.start_time.setText(str(self.motion_data[0][0]))  # First time point
                    self.end_time.setText(str(self.motion_data[-1][0]))   # Last time point
                
                # Update preview table
                self.update_table()
                
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error loading motion data: {str(e)}")
            self.status_bar.showMessage("Error loading motion data ")
            
    def calculate_grf(self):
        try:
            # Get parameters
            start_time = float(self.start_time.text())
            end_time = float(self.end_time.text())
            cutoff_freq = float(self.cutoff_freq.text())
            force_threshold = float(self.force_threshold.text())
            
            # Update status
            self.grf_status.setText("Calculating...")
            self.grf_status.setStyleSheet("color: #ffa000;")
            self.calculate_btn.setEnabled(False)
            QApplication.processEvents()  # Update UI
            
            # TODO: Implement actual GRF calculation here
            # This will need to be implemented based on the MATLAB code
            
            # Update status on completion
            self.grf_status.setText(" Calculation complete")
            self.grf_status.setStyleSheet("color: #66bb6a;")
            self.calculation_done = True
            self.status_bar.showMessage("GRF calculation completed successfully ")
            
        except ValueError as e:
            QMessageBox.critical(self, "Error", "Please enter valid numbers for all parameters")
            self.status_bar.showMessage("Invalid parameter values ")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error calculating GRF: {str(e)}")
            self.status_bar.showMessage("GRF calculation failed ")
        finally:
            self.calculate_btn.setEnabled(True)
    
    def update_table(self):
        if self.motion_data is None or self.motion_headers is None:
            return
            
        try:
            # Get time range
            start_time = float(self.start_time.text()) if self.start_time.text() else self.motion_data[0][0]
            end_time = float(self.end_time.text()) if self.end_time.text() else self.motion_data[-1][0]
            
            # Find indices within time range
            time_column = np.array(self.motion_data)[:, 0]
            mask = (time_column >= start_time) & (time_column <= end_time)
            filtered_data = np.array(self.motion_data)[mask]
            
            # Set up table
            self.table.setRowCount(len(filtered_data))
            self.table.setColumnCount(len(self.motion_headers))
            self.table.setHorizontalHeaderLabels(self.motion_headers)
            
            # Fill data
            for i in range(len(filtered_data)):
                for j in range(len(self.motion_headers)):
                    item = QTableWidgetItem(f"{filtered_data[i][j]:.6f}")
                    item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                    self.table.setItem(i, j, item)
            
            # Adjust column widths
            self.table.resizeColumnsToContents()
            
        except ValueError:
            # If time values are invalid, show all data
            self.table.setRowCount(len(self.motion_data))
            self.table.setColumnCount(len(self.motion_headers))
            self.table.setHorizontalHeaderLabels(self.motion_headers)
            
            for i in range(len(self.motion_data)):
                for j in range(len(self.motion_headers)):
                    item = QTableWidgetItem(f"{self.motion_data[i][j]:.6f}")
                    item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                    self.table.setItem(i, j, item)
            
            self.table.resizeColumnsToContents()
        
def main():
    app = QApplication(sys.argv)
    
    # Set application-wide style
    app.setStyle("Fusion")
    
    # Create and show the GUI
    window = OpenGRFGui()
    window.show()
    
    sys.exit(app.exec())

if __name__ == "__main__":
    main()

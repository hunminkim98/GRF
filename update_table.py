from PyQt6.QtWidgets import QTableWidgetItem
from PyQt6.QtCore import Qt
import numpy as np

def motion_update_table(table, motion_data, motion_headers, start_time_text=None, end_time_text=None):
    """
    Update the table with motion data filtered by time range.
    
    Args:
        table: QTableWidget instance to update
        motion_data: List of motion data
        motion_headers: List of column headers
        start_time_text: String representing start time (optional)
        end_time_text: String representing end time (optional)
    """
    if motion_data is None or motion_headers is None:
        return
        
    try:
        # Get time range
        start_time = float(start_time_text) if start_time_text else motion_data[0][0]
        end_time = float(end_time_text) if end_time_text else motion_data[-1][0]
        
        # Find indices within time range
        time_column = np.array(motion_data)[:, 0]
        mask = (time_column >= start_time) & (time_column <= end_time)
        filtered_data = np.array(motion_data)[mask]
        
        # Set up table
        table.setRowCount(len(filtered_data))
        table.setColumnCount(len(motion_headers))
        table.setHorizontalHeaderLabels(motion_headers)
        
        # Fill data
        for i in range(len(filtered_data)):
            for j in range(len(motion_headers)):
                item = QTableWidgetItem(f"{filtered_data[i][j]:.6f}")
                item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                table.setItem(i, j, item)
        
        # Adjust column widths
        table.resizeColumnsToContents()
        
    except ValueError:
        # If time values are invalid, show all data
        table.setRowCount(len(motion_data))
        table.setColumnCount(len(motion_headers))
        table.setHorizontalHeaderLabels(motion_headers)
        
        for i in range(len(motion_data)):
            for j in range(len(motion_headers)):
                item = QTableWidgetItem(f"{motion_data[i][j]:.6f}")
                item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                table.setItem(i, j, item)
        
        table.resizeColumnsToContents()

def position_update_table(table, position_data):
    """
    Update the table with contact position data and ground height values.
    Args:
        table: QTableWidget to update
        position_data: Dictionary containing position data and ground height values
    """
    # Clear the table
    table.clear()
    table.setRowCount(0)
    
    # Set up headers
    headers = ['Contact Point', 'X', 'Y', 'Z']
    table.setColumnCount(len(headers))
    table.setHorizontalHeaderLabels(headers)
    
    # Add data
    row = 0
    for key, value in sorted(position_data.items()):
        table.insertRow(row)
        table.setItem(row, 0, QTableWidgetItem(key))
        
        # Handle both coordinate lists and single ground height values
        if isinstance(value, (list, tuple)):
            # Contact position coordinates (X, Y, Z)
            for j, coord in enumerate(value):
                table.setItem(row, j+1, QTableWidgetItem(f"{coord:.6f}"))
        else:
            # Ground height value (single float) - display in Y column
            table.setItem(row, 1, QTableWidgetItem("-"))
            table.setItem(row, 2, QTableWidgetItem(f"{value:.6f}"))  # Y column
            table.setItem(row, 3, QTableWidgetItem("-"))
        row += 1
    
    # Adjust column widths
    table.resizeColumnsToContents()

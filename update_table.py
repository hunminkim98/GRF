from PyQt6.QtWidgets import QTableWidgetItem, QTableWidget
from PyQt6.QtCore import Qt
import numpy as np

def _setup_table(table, headers):
    """
    공통적인 테이블 초기화 작업을 수행하는 헬퍼 함수
    
    Args:
        table: QTableWidget to setup
        headers: List of column headers
    """
    table.clear()
    table.setRowCount(0)
    table.setColumnCount(len(headers))
    table.setHorizontalHeaderLabels(headers)

def _create_table_item(value, alignment=Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter):
    """
    테이블 아이템 생성을 위한 헬퍼 함수
    
    Args:
        value: Value to display
        alignment: Text alignment flags
    Returns:
        QTableWidgetItem with formatted value and alignment
    """
    if isinstance(value, (int, float)):
        item = QTableWidgetItem(f"{value:.6f}")
    else:
        item = QTableWidgetItem(str(value))
    item.setTextAlignment(alignment)
    return item

def _get_or_create_table(tab_widget, tab_name):
    """
    탭에서 테이블을 가져오거나 새로 생성하는 헬퍼 함수
    
    Args:
        tab_widget: QTabWidget instance
        tab_name: Name of the tab to find or create
    Returns:
        QTableWidget instance
    """
    # 기존 탭 찾기
    for i in range(tab_widget.count()):
        if tab_widget.tabText(i) == tab_name:
            return tab_widget.widget(i)
    
    # 새 테이블 생성
    table = QTableWidget()
    tab_widget.addTab(table, tab_name)
    return table

def update_table(tab_widget, data_type, data, **kwargs):
    """
    테이블 업데이트를 위한 통합 함수
    
    Args:
        tab_widget: QTabWidget to update
        data_type: Type of data to display ('motion', 'position', or 'kinematics')
        data: Data to display
        **kwargs: Additional arguments specific to each data type
            - motion: motion_headers, start_time_text, end_time_text
            - position: No additional arguments
            - kinematics: No additional arguments
    """
    if data_type == 'motion':
        table = _get_or_create_table(tab_widget, 'Motion Data')
        motion_headers = kwargs.get('motion_headers')
        start_time_text = kwargs.get('start_time_text')
        end_time_text = kwargs.get('end_time_text')
        
        if data is None or motion_headers is None:
            return
            
        try:
            # Get time range
            start_time = float(start_time_text) if start_time_text else data[0][0]
            end_time = float(end_time_text) if end_time_text else data[-1][0]
            
            # Find indices within time range
            time_column = np.array(data)[:, 0]
            mask = (time_column >= start_time) & (time_column <= end_time)
            filtered_data = np.array(data)[mask]
            
            # Set up table
            _setup_table(table, motion_headers)
            table.setRowCount(len(filtered_data))
            
            # Fill data
            for i in range(len(filtered_data)):
                for j in range(len(motion_headers)):
                    table.setItem(i, j, _create_table_item(filtered_data[i][j]))
                    
        except ValueError:
            # If time values are invalid, show all data
            _setup_table(table, motion_headers)
            table.setRowCount(len(data))
            
            for i in range(len(data)):
                for j in range(len(motion_headers)):
                    table.setItem(i, j, _create_table_item(data[i][j]))
    
    elif data_type == 'position':
        table = _get_or_create_table(tab_widget, 'Contact Positions')
        # Set up headers
        headers = ['Contact Point', 'X', 'Y', 'Z']
        _setup_table(table, headers)
        
        # Add data
        for key, value in sorted(data.items()):
            row = table.rowCount()
            table.insertRow(row)
            table.setItem(row, 0, _create_table_item(key, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter))
            
            if isinstance(value, (list, tuple)):
                for j, coord in enumerate(value):
                    table.setItem(row, j+1, _create_table_item(coord))
            else:
                table.setItem(row, 1, _create_table_item("-"))
                table.setItem(row, 2, _create_table_item(value))
                table.setItem(row, 3, _create_table_item("-"))
    
    elif data_type == 'kinematics':
        table = _get_or_create_table(tab_widget, 'Kinematics Data')
        # Set up headers
        headers = ['Time', 
                  'Calcn_L_X', 'Calcn_L_Y', 'Calcn_L_Z',
                  'Calcn_L_VX', 'Calcn_L_VY', 'Calcn_L_VZ',
                  'Calcn_L_AX', 'Calcn_L_AY', 'Calcn_L_AZ',
                  'Calcn_R_X', 'Calcn_R_Y', 'Calcn_R_Z',
                  'Calcn_R_VX', 'Calcn_R_VY', 'Calcn_R_VZ',
                  'Calcn_R_AX', 'Calcn_R_AY', 'Calcn_R_AZ',
                  'Hand_L_X', 'Hand_L_Y', 'Hand_L_Z',
                  'Hand_R_X', 'Hand_R_Y', 'Hand_R_Z']
        
        _setup_table(table, headers)
        
        # Get number of time points
        n_points = len(data['time'])
        table.setRowCount(n_points)
        
        # Fill data for each time point
        for i in range(n_points):
            # Time
            table.setItem(i, 0, _create_table_item(data['time'][i]))
            
            # Left calcn (position, velocity, acceleration)
            for j, val in enumerate(data['calcn_l']['pos'][i]):
                table.setItem(i, 1+j, _create_table_item(val))
            for j, val in enumerate(data['calcn_l']['vel'][i]):
                table.setItem(i, 4+j, _create_table_item(val))
            for j, val in enumerate(data['calcn_l']['acc'][i]):
                table.setItem(i, 7+j, _create_table_item(val))
                
            # Right calcn (position, velocity, acceleration)
            for j, val in enumerate(data['calcn_r']['pos'][i]):
                table.setItem(i, 10+j, _create_table_item(val))
            for j, val in enumerate(data['calcn_r']['vel'][i]):
                table.setItem(i, 13+j, _create_table_item(val))
            for j, val in enumerate(data['calcn_r']['acc'][i]):
                table.setItem(i, 16+j, _create_table_item(val))
                
            # Hands (position only)
            for j, val in enumerate(data['hand_l']['pos'][i]):
                table.setItem(i, 19+j, _create_table_item(val))
            for j, val in enumerate(data['hand_r']['pos'][i]):
                table.setItem(i, 22+j, _create_table_item(val))
    
    # Adjust column widths
    table.resizeColumnsToContents()

# 기존 함수들을 새로운 통합 함수를 사용하도록 수정
def motion_update_table(tab_widget, motion_data, motion_headers, start_time_text=None, end_time_text=None):
    """
    Update the table with motion data filtered by time range.
    """
    update_table(tab_widget, 'motion', motion_data, 
                motion_headers=motion_headers,
                start_time_text=start_time_text,
                end_time_text=end_time_text)

def position_update_table(tab_widget, position_data):
    """
    Update the table with contact position data and ground height values.
    """
    update_table(tab_widget, 'position', position_data)

def kinematics_update_table(tab_widget, kinematics_data):
    """
    Update the table with kinematics data (position, velocity, acceleration).
    """
    update_table(tab_widget, 'kinematics', kinematics_data)

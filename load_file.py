import numpy as np

def load_mot(fname):
    """
    Load motion data from a .mot file
    
    Parameters:
        fname (str): Path to the .mot file
        
    Returns:
        tuple: (data, headers)
            - data: numpy array containing the motion data
            - headers: list of column headers
    """
    with open(fname, 'r') as fid:
        # Read until endheader
        while True:
            line = fid.readline()
            if not line:  # EOF check
                break
            # Similar to MATLAB's sscanf(line, '%s')
            line = line.strip()
            if line == 'endheader':
                break
        
        # Read column headers (similar to textscan(line, '%s'))
        line = fid.readline()
        headers = [x for x in line.strip().split() if x]  # Remove empty strings
        
        # Read data (similar to MATLAB's while(~feof))
        data_list = []
        while True:
            line = fid.readline()
            if not line:  # EOF
                break
            # Convert line to float numbers (similar to textscan(line, '%f'))
            row = [float(x) for x in line.strip().split()]
            data_list.append(row)
        
        # Convert to numpy array
        data = np.array(data_list)
        
    return data, headers

def load_sto(fname):
    """
    Load data from a .sto file
    
    Parameters:
        fname (str): Path to the .sto file
        
    Returns:
        tuple: (data, headers)
            - data: numpy array containing the data
            - headers: list of column headers
    """
    with open(fname, 'r') as fid:
        # Skip first 4 lines
        for _ in range(4):
            line = fid.readline()
            
        # Read nColumns
        line = fid.readline().strip()
        n_columns = int(line.split('=')[1])
        
        # Skip next 2 lines
        for _ in range(2):
            line = fid.readline()
            
        # Read until endheader
        while True:
            line = fid.readline().strip()
            if line == 'endheader':
                break
                
        # Read column headers (similar to textscan(line, '%s'))
        line = fid.readline()
        headers = [x for x in line.strip().split() if x]  # Remove empty strings
        
        # Read data
        data_list = []
        while True:
            line = fid.readline()
            if not line:  # EOF
                break
            # Convert line to float numbers (similar to textscan(line, '%f'))
            row = [float(x) for x in line.strip().split()]
            data_list.append(row)
            
        # Convert to numpy array
        data = np.array(data_list)
        
    return data, headers

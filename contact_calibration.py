import opensim
import numpy as np
import os

def calibrate_contact_model(model_path, ik_result_path, time_range=None, force_threshold=400, initial_delta=0.01, min_delta=0.001, cutoff_freq=8.0):
    """
    Calibrates the contact elements of an OpenSim model by iteratively adjusting sphere positions.
    
    Args:
        model_path (str): Path to the OpenSim model with contact elements
        ik_result_path (str): Path to the IK results file (.mot)
        time_range (tuple): Start and end time for analysis (default: None, uses full range)
        force_threshold (float): Minimum required vertical force in N (default: 400)
        initial_delta (float): Initial position adjustment increment in meters (default: 0.01)
        min_delta (float): Minimum position adjustment increment in meters (default: 0.001)
        cutoff_freq (float): Low-pass filter cutoff frequency in Hz (default: 8.0)
    
    Returns:
        tuple: (calibrated_model_path, F_min_L, F_min_R)
            - calibrated_model_path (str): Path to the calibrated model file
            - F_min_L (ndarray): Minimum forces for left foot spheres
            - F_min_R (ndarray): Minimum forces for right foot spheres
    """
    # Load the model
    model = opensim.Model(model_path)
    
    # Create and configure the analyze tool
    analyze_tool = opensim.AnalyzeTool()
    analyze_tool.setModel(model)
    analyze_tool.setModelFilename(model_path)
    analyze_tool.setCoordinatesFileName(ik_result_path)
    analyze_tool.setLowpassCutoffFrequency(cutoff_freq)
    analyze_tool.setSolveForEquilibrium(False)
    
    if time_range:
        analyze_tool.setStartTime(time_range[0])
        analyze_tool.setFinalTime(time_range[1])
    
    # Set results directory
    setup_dir = os.path.dirname(model_path)
    force_reporter_dir = os.path.join(setup_dir, 'ForceReporter')
    os.makedirs(force_reporter_dir, exist_ok=True)
    analyze_tool.setResultsDir(force_reporter_dir)
    
    # Set up force reporter
    force_reporter = opensim.ForceReporter()
    if time_range:
        force_reporter.setStartTime(time_range[0])
        force_reporter.setEndTime(time_range[1])
    
    analyze_tool.getAnalysisSet().cloneAndAppend(force_reporter)
    
    # Get the setup file directory
    setup_file = os.path.join(setup_dir, 'Setup_ForceReporter.xml')
    analyze_tool.printToXML(setup_file)
    
    iteration = 0
    while True:  # Continue until all forces are ok
        # Get all contact spheres
        contact_spheres = []
        for i in range(1, 15):  # 1 to 14
            for side in ['R', 'L']:
                sphere_name = f'Sphere_Foot_{i}_{side}'
                try:
                    # Try with ContactGeometry path
                    sphere = model.getContactGeometrySet().get(sphere_name)
                    if not sphere:
                        # Try with model name prefix
                        model_name = model.getName()
                        sphere = model.getContactGeometrySet().get(f'{model_name}/{sphere_name}')
                    if not sphere:
                        raise Exception(f"Could not find sphere {sphere_name}")
                    contact_spheres.append(sphere)
                except Exception as e:
                    print(f"Error getting sphere {sphere_name}: {str(e)}")
                    raise
        
        # Run the analysis
        iter_tool = opensim.AnalyzeTool(setup_file)
        iter_tool.setResultsDir(force_reporter_dir)
        if time_range:
            iter_tool.setStartTime(time_range[0])
            iter_tool.setFinalTime(time_range[1])
        iter_tool.run()
        
        # Get force file path
        force_file = os.path.join(force_reporter_dir, '_ForceReporter_forces.sto')
        
        if not os.path.exists(force_file):
            print(f"Force file not found at: {force_file}")
            print(f"Contents of {force_reporter_dir}:")
            for f in os.listdir(force_reporter_dir):
                print(f"  {f}")
            raise FileNotFoundError(f"Force reporter output not found: {force_file}")
        
        # Initialize force tracking arrays
        F_min_L = np.zeros(14)  # Minimum forces for left foot
        F_min_R = np.zeros(14)  # Minimum forces for right foot
        sphere_flags = np.zeros(28)  # Flags for each sphere (14 per foot)
        
        # Check forces and adjust positions
        all_forces_ok = True
        for i, sphere in enumerate(contact_spheres):
            # Get current position
            pos = sphere.get_location()
            
            # Get force and check threshold
            sphere_name = sphere.getName()
            force = get_vertical_force(force_file, sphere_name, get_min=True)
            
            # Get sphere index and side
            sphere_num = int(sphere_name.split('_')[2])
            side = sphere_name[-1]
            
            # Use different threshold for front spheres (1-4)
            threshold = force_threshold / 3 if sphere_num <= 4 else force_threshold
            
            if abs(force) < threshold:
                # Adjust Y position down
                new_pos = opensim.Vec3(pos.get(0), pos.get(1) - delta, pos.get(2))
                sphere.set_location(new_pos)
                sphere_flags[i] = 0
                all_forces_ok = False
            else:
                # Store minimum force
                if side == 'L':
                    F_min_L[sphere_num - 1] = force
                else:
                    F_min_R[sphere_num - 1] = force
                sphere_flags[i] = 1
        
        # Print iteration info
        print(f"Iteration {iteration}:")
        print("Left foot min forces:", F_min_L)
        print("Right foot min forces:", F_min_R)
        print("Sphere flags:", sphere_flags)
        
        # Save model if forces are ok or continue
        if all_forces_ok:
            model.finalizeConnections()
            calibrated_model_path = os.path.join(setup_dir, 'calibrated_model.osim')
            model.printToXML(calibrated_model_path)
            return calibrated_model_path, F_min_L, F_min_R  # Return minimum forces for analysis
            
        iteration += 1

def get_vertical_force(force_file, sphere_name, get_min=True):
    """
    Gets the vertical (Y) component of the contact force for a given sphere.
    
    Args:
        force_file (str): Path to the ForceReporter results file (_ForceReporter_forces.sto)
        sphere_name (str): Name of the sphere (e.g., 'Foot_1_L')
        get_min (bool): If True, returns minimum force over all frames. If False, returns first frame force.
        
    Returns:
        float: Vertical force value (minimum if get_min=True, otherwise first frame)
    """
    # Load the force reporter results
    with open(force_file, 'r') as f:
        # Skip header until endheader
        line = f.readline()
        headers = []
        while 'endheader' not in line:
            if line.startswith('nRows='):
                nRows = int(line.split('=')[1])
            elif line.startswith('nColumns='):
                nColumns = int(line.split('=')[1])
            line = f.readline()
        
        # Read column headers
        headers = f.readline().strip().split('\t')
        
        # Find the column index for the vertical force
        force_column = -1
        sphere_name = sphere_name.replace('Sphere_', '')  # Remove 'Sphere_' prefix for force column
        for i, header in enumerate(headers):
            if header == f'ForceGround_{sphere_name}.ground.force.Y':
                force_column = i
                break
        
        if force_column == -1:
            raise ValueError(f"Force column not found for sphere {sphere_name}")
        
        # Read all force values
        forces = []
        for line in f:
            values = line.strip().split('\t')
            forces.append(float(values[force_column]))
        
        # Return minimum force if requested, otherwise first frame
        if get_min:
            return min(forces)
        else:
            return forces[0]

import opensim
import numpy as np
import os
from load_file import load_sto

def calibrate_contact_model(model_path, ik_result_path, time_range=None, force_threshold=400, delta=0.001, cutoff_freq=8.0):
    """
    Calibrates the contact elements of an OpenSim model by iteratively adjusting sphere positions.
    Each sphere is adjusted independently based on its force threshold.
    
    Args:
        model_path (str): Path to the OpenSim model with contact elements
        ik_result_path (str): Path to the IK results file (.mot)
        time_range (tuple): Start and end time for analysis (default: None, uses full range)
        force_threshold (float): Minimum required vertical force in N (default: 400)
        delta (float): Position adjustment increment in meters (default: 0.001)
        cutoff_freq (float): Low-pass filter cutoff frequency in Hz (default: 8.0)
    
    Returns:
        tuple: (calibrated_model_path, F_min_L, F_min_R)
            - calibrated_model_path (str): Path to the calibrated model file
            - F_min_L (ndarray): Minimum forces for left foot spheres
            - F_min_R (ndarray): Minimum forces for right foot spheres
    """

    # log basic parameters
    print(f"Model path: {model_path}")
    print(f"IK result path: {ik_result_path}")
    print(f"Time range: {time_range}")
    print(f"Force threshold: {force_threshold}")
    print(f"Delta: {delta}")
    print(f"Cutoff frequency: {cutoff_freq}")

    # Load the model
    model = opensim.Model(model_path)
    
    # Create and configure the analyze tool
    analyze_tool = opensim.AnalyzeTool()
    analyze_tool.setModel(model)
    analyze_tool.setModelFilename(model_path)
    analyze_tool.setCoordinatesFileName(ik_result_path)
    analyze_tool.setLowpassCutoffFrequency(cutoff_freq)
    analyze_tool.setSolveForEquilibrium(True)
    
    if time_range:
        analyze_tool.setStartTime(time_range[0])
        analyze_tool.setFinalTime(time_range[1])
    
    # Set results directory
    setup_dir = os.path.dirname(model_path)
    force_reporter_dir = os.path.join(setup_dir, 'ForceReporter')
    BodyKinematics_dir = os.path.join(setup_dir, 'BK')
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
    
    # Get kinematics file paths
    kinematics_pos_path = os.path.join(BodyKinematics_dir, '_BodyKinematics_pos_global.sto')
    kinematics_acc_path = os.path.join(BodyKinematics_dir, '_BodyKinematics_acc_global.sto')
    
    # Find contact positions
    position_r, position_l = find_contact_positions(kinematics_pos_path, kinematics_acc_path)
    print(f"Position R: {position_r}")
    print(f"Position L: {position_l}")

    # Set calibrated model path
    calibrated_model_path = os.path.join(setup_dir, 'calibrated_model.osim')
    
    # Get all contact spheres and store their initial positions
    sphere_positions = {}  # Dictionary to store each sphere's position
    contact_spheres = []
    for i in range(1, 15):  # 1 to 14
        for side in ['R', 'L']:
            sphere_name = f'Sphere_Foot_{i}_{side}'
            try:
                sphere = model.getContactGeometrySet().get(sphere_name)
                if not sphere:
                    model_name = model.getName()
                    sphere = model.getContactGeometrySet().get(f'{model_name}/{sphere_name}')
                if not sphere:
                    raise Exception(f"Could not find sphere {sphere_name}")
                contact_spheres.append(sphere)
                # Store initial position
                pos = sphere.get_location()
                sphere_positions[sphere_name] = opensim.Vec3(pos.get(0), pos.get(1), pos.get(2))
            except Exception as e:
                print(f"Error getting sphere {sphere_name}: {str(e)}")
                raise
    
    iteration = 0
    while True:
        # Initialize force tracking arrays
        F_min_L = np.zeros(14)
        F_min_R = np.zeros(14)
        sphere_flags = np.zeros(28)
        all_forces_ok = True
        
        # Run ForceReporter to get current forces
        model.finalizeConnections()
        model.printToXML(calibrated_model_path)
        
        iter_tool = opensim.AnalyzeTool(setup_file)
        iter_tool.setModelFilename(calibrated_model_path)
        iter_tool.setResultsDir(force_reporter_dir)
        if time_range:
            iter_tool.setStartTime(time_range[0])
            iter_tool.setFinalTime(time_range[1])
        iter_tool.run()
        
        # Load force data
        force_file = os.path.join(force_reporter_dir, '_ForceReporter_forces.sto')
        if not os.path.exists(force_file):
            raise FileNotFoundError(f"Force reporter output not found: {force_file}")
        force_data, force_headers = load_sto(force_file)
        
        # Process each sphere independently
        for i, sphere in enumerate(contact_spheres):
            sphere_name = sphere.getName()
            force_col = f'ForceGround_{sphere_name.replace("Sphere_", "")}.ground.force.Y'
            force_idx = force_headers.index(force_col)
            
            # Get current position from stored positions
            current_pos = sphere_positions[sphere_name]
            
            # Get force at specific position based on side
            side = sphere_name[-1]
            sphere_num = int(sphere_name.split('_')[2])  # Changed to match MATLAB indexing
            if side == 'L':
                force = force_data[position_l, force_idx]
            else:
                force = force_data[position_r, force_idx]
            
            # Use different threshold for front spheres (1-4)
            threshold = force_threshold / 3 if sphere_num <= 4 else force_threshold
            
            if abs(force) < threshold:
                # Get original Y position for logging
                old_y = current_pos.get(1)
                
                # Adjust Y position down for this specific sphere
                new_y = old_y - delta
                new_pos = opensim.Vec3(current_pos.get(0), new_y, current_pos.get(2))
                sphere.set_location(new_pos)
                sphere_positions[sphere_name] = new_pos  # Update stored position
                
                # Print position and force information
                print(f"  {sphere_name}:")
                print(f"    Y position adjusted by {delta*1000:.2f} mm (from {old_y*1000:.2f} mm to {new_y*1000:.2f} mm)")
                print(f"    Current force: {force:.2f} N (threshold: {threshold:.2f} N)")
                
                sphere_flags[i] = 0
                all_forces_ok = False
            else:
                # Store minimum force
                if side == 'L':
                    F_min_L[sphere_num - 1] = min(force_data[:, force_idx])
                else:
                    F_min_R[sphere_num - 1] = min(force_data[:, force_idx])
                sphere_flags[i] = 1
                print(f"  {sphere_name}: Force {force:.2f} N meets threshold ({threshold:.2f} N)")
        
        # Print iteration summary
        print(f"\nIteration {iteration} Summary:")
        print("Left foot min forces:", F_min_L)
        print("Right foot min forces:", F_min_R)
        print("Sphere flags:", sphere_flags)
        
        # Save model and check if done
        model.finalizeConnections()
        model.printToXML(calibrated_model_path)
        
        if all_forces_ok:
            return calibrated_model_path, F_min_L, F_min_R
            
        iteration += 1

def find_contact_positions(kinematics_pos_path, kinematics_acc_path):
    """
    Calculate the frame indices where feet are closest to ground with minimal acceleration
    
    Args:
        kinematics_pos_path (str): Path to BodyKinematics_pos_global.sto
        kinematics_acc_path (str): Path to BodyKinematics_acc_global.sto
        
    Returns:
        tuple: (position_r, position_l) frame indices
    """
    # Load position and acceleration data
    pos_data, pos_headers = load_sto(kinematics_pos_path)
    acc_data, acc_headers = load_sto(kinematics_acc_path)
    
    # Find column indices for calcn_r_Y and calcn_l_Y
    calcn_r_y_idx = pos_headers.index('calcn_r_Y')
    calcn_l_y_idx = pos_headers.index('calcn_l_Y')
    
    # Get position data
    p_calcn_r_y = pos_data[:, calcn_r_y_idx]
    p_calcn_l_y = pos_data[:, calcn_l_y_idx]
    
    # Find frames where position is near minimum
    min_r = np.min(p_calcn_r_y)
    min_l = np.min(p_calcn_l_y)
    
    range_pos_r = p_calcn_r_y < (min_r + min_r * 1/1000)
    range_pos_l = p_calcn_l_y < (min_l + min_l * 1/1000)
    
    min_p_calcn_r_y = np.where(range_pos_r)[0]
    min_p_calcn_l_y = np.where(range_pos_l)[0]
    
    # Get acceleration data for those frames
    a_calcn_r_y = acc_data[:, acc_headers.index('calcn_r_Y')]
    a_calcn_l_y = acc_data[:, acc_headers.index('calcn_l_Y')]
    
    # Find frame with minimum acceleration among minimum height frames
    sub_r_acc = a_calcn_r_y[min_p_calcn_r_y]
    sub_l_acc = a_calcn_l_y[min_p_calcn_l_y]
    
    pos_r = np.argmin(np.abs(sub_r_acc))
    pos_l = np.argmin(np.abs(sub_l_acc))
    
    # Convert back to original frame indices
    position_r = min_p_calcn_r_y[pos_r]
    position_l = min_p_calcn_l_y[pos_l]
    
    return position_r, position_l

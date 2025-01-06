import opensim
import numpy as np

# Ground height variables
Ground_R_Foot_Height = 0.0
Ground_L_Foot_Height = 0.0
Ground_R_Hand_Height = 0.0
Ground_L_Hand_Height = 0.0

def calculate_contact_positions(model, heel_shift, verbose=False):
    if verbose:
        print("Starting contact positions calculation...")
        print(f"Using heel_shift value: {heel_shift}")
    
    # Get scale factors for each body
    if verbose:
        print("Getting body scale factors...")
    
    calcn_r = model.getBodySet().get('calcn_r')
    calcn_l = model.getBodySet().get('calcn_l')
    toes_r = model.getBodySet().get('toes_r')
    toes_l = model.getBodySet().get('toes_l')
    
    # Convert Vec3 to numpy array for Python (adjusting for MATLAB's 1-based indexing)
    # In MATLAB: (1) = x, (2) = y, (3) = z
    # In Python: [0] = x, [1] = y, [2] = z
    calcn_r_sf = np.array([
        calcn_r.get_attached_geometry(0).get_scale_factors().get(0),  # x
        calcn_r.get_attached_geometry(0).get_scale_factors().get(1),  # y
        calcn_r.get_attached_geometry(0).get_scale_factors().get(2)   # z
    ])
    calcn_l_sf = np.array([
        calcn_l.get_attached_geometry(0).get_scale_factors().get(0),
        calcn_l.get_attached_geometry(0).get_scale_factors().get(1),
        calcn_l.get_attached_geometry(0).get_scale_factors().get(2)
    ])
    toes_r_sf = np.array([
        toes_r.get_attached_geometry(0).get_scale_factors().get(0),
        toes_r.get_attached_geometry(0).get_scale_factors().get(1),
        toes_r.get_attached_geometry(0).get_scale_factors().get(2)
    ])
    toes_l_sf = np.array([
        toes_l.get_attached_geometry(0).get_scale_factors().get(0),
        toes_l.get_attached_geometry(0).get_scale_factors().get(1),
        toes_l.get_attached_geometry(0).get_scale_factors().get(2)
    ])
    
    if verbose:
        print("Scale factors obtained:")
        print(f"Right calcaneus: {calcn_r_sf}")
        print(f"Left calcaneus: {calcn_l_sf}")
        print(f"Right toes: {toes_r_sf}")
        print(f"Left toes: {toes_l_sf}")
        print("\nCalculating sphere positions...")
    
    # Calculate positions for contact spheres (matching MATLAB's calculations exactly)
    pos_sp1r = [(0 + heel_shift) * calcn_r_sf[0], 0.03 * calcn_r_sf[1], -0.01 * calcn_r_sf[2]]
    pos_sp1l = [(0 + heel_shift) * calcn_l_sf[0], 0.03 * calcn_l_sf[1], -0.01 * calcn_l_sf[2]]
    pos_sp2r = [(0 + heel_shift) * calcn_r_sf[0], 0.03 * calcn_r_sf[1], 0.01 * calcn_r_sf[2]]
    pos_sp2l = [(0 + heel_shift) * calcn_l_sf[0], 0.03 * calcn_l_sf[1], 0.01 * calcn_l_sf[2]]
    pos_sp3r = [(0.035 + heel_shift) * calcn_r_sf[0], 0.03 * calcn_r_sf[1], -0.02 * calcn_r_sf[2]]
    pos_sp3l = [(0.035 + heel_shift) * calcn_l_sf[0], 0.03 * calcn_l_sf[1], -0.02 * calcn_l_sf[2]]
    pos_sp4r = [(0.035 + heel_shift) * calcn_r_sf[0], 0.03 * calcn_r_sf[1], 0.02 * calcn_r_sf[2]]
    pos_sp4l = [(0.035 + heel_shift) * calcn_l_sf[0], 0.03 * calcn_l_sf[1], 0.02 * calcn_l_sf[2]]
    pos_sp5r = [0.025 * toes_r_sf[0], 0.03 * toes_r_sf[1], -0.01 * toes_r_sf[2]]
    pos_sp5l = [0.025 * toes_l_sf[0], 0.03 * toes_l_sf[1], -0.02 * toes_l_sf[2]]
    pos_sp6r = [0.025 * toes_r_sf[0], 0.03 * toes_r_sf[1], 0.02 * toes_r_sf[2]]
    pos_sp6l = [0.025 * toes_l_sf[0], 0.03 * toes_l_sf[1], 0.01 * toes_l_sf[2]]
    pos_sp7r = [0.07 * calcn_r_sf[0], 0.03 * calcn_r_sf[1], -0.015 * calcn_r_sf[2]]
    pos_sp7l = [0.07 * calcn_l_sf[0], 0.03 * calcn_l_sf[1], -0.035 * calcn_l_sf[2]]
    pos_sp8r = [0.07 * calcn_r_sf[0], 0.03 * calcn_r_sf[1], 0.035 * calcn_r_sf[2]]
    pos_sp8l = [0.07 * calcn_l_sf[0], 0.03 * calcn_l_sf[1], 0.015 * calcn_l_sf[2]]
    pos_sp9r = [0.105 * calcn_r_sf[0], 0.03 * calcn_r_sf[1], -0.005 * calcn_r_sf[2]]
    pos_sp9l = [0.105 * calcn_l_sf[0], 0.03 * calcn_l_sf[1], -0.045 * calcn_l_sf[2]]
    pos_sp10r = [0.105 * calcn_r_sf[0], 0.03 * calcn_r_sf[1], 0.045 * calcn_r_sf[2]]
    pos_sp10l = [0.105 * calcn_l_sf[0], 0.03 * calcn_l_sf[1], 0.005 * calcn_l_sf[2]]
    pos_sp11r = [0.14 * calcn_r_sf[0], 0.03 * calcn_r_sf[1], -0.005 * calcn_r_sf[2]]
    pos_sp11l = [0.14 * calcn_l_sf[0], 0.03 * calcn_l_sf[1], -0.045 * calcn_l_sf[2]]
    pos_sp12r = [0.14 * calcn_r_sf[0], 0.03 * calcn_r_sf[1], 0.045 * calcn_r_sf[2]]
    pos_sp12l = [0.14 * calcn_l_sf[0], 0.03 * calcn_l_sf[1], 0.005 * calcn_l_sf[2]]
    pos_sp13r = [0.0 * toes_r_sf[0], 0.03 * toes_r_sf[1], -0.005 * toes_r_sf[2]]
    pos_sp13l = [0.0 * toes_l_sf[0], 0.03 * toes_l_sf[1], -0.03 * toes_l_sf[2]]
    pos_sp14r = [0.0 * toes_r_sf[0], 0.03 * toes_r_sf[1], 0.03 * toes_r_sf[2]]
    pos_sp14l = [0.0 * toes_l_sf[0], 0.03 * toes_l_sf[1], 0.005 * toes_l_sf[2]]
    
    # Return all positions as a dictionary
    result = {
        'pos_sp1r': pos_sp1r, 'pos_sp1l': pos_sp1l,
        'pos_sp2r': pos_sp2r, 'pos_sp2l': pos_sp2l,
        'pos_sp3r': pos_sp3r, 'pos_sp3l': pos_sp3l,
        'pos_sp4r': pos_sp4r, 'pos_sp4l': pos_sp4l,
        'pos_sp5r': pos_sp5r, 'pos_sp5l': pos_sp5l,
        'pos_sp6r': pos_sp6r, 'pos_sp6l': pos_sp6l,
        'pos_sp7r': pos_sp7r, 'pos_sp7l': pos_sp7l,
        'pos_sp8r': pos_sp8r, 'pos_sp8l': pos_sp8l,
        'pos_sp9r': pos_sp9r, 'pos_sp9l': pos_sp9l,
        'pos_sp10r': pos_sp10r, 'pos_sp10l': pos_sp10l,
        'pos_sp11r': pos_sp11r, 'pos_sp11l': pos_sp11l,
        'pos_sp12r': pos_sp12r, 'pos_sp12l': pos_sp12l,
        'pos_sp13r': pos_sp13r, 'pos_sp13l': pos_sp13l,
        'pos_sp14r': pos_sp14r, 'pos_sp14l': pos_sp14l,
        'Ground_R_Foot_Height': Ground_R_Foot_Height,
        'Ground_L_Foot_Height': Ground_L_Foot_Height,
        'Ground_R_Hand_Height': Ground_R_Hand_Height,
        'Ground_L_Hand_Height': Ground_L_Hand_Height
    }
    
    if verbose:
        print("All sphere positions calculated successfully!")
        print("\nAll calculated positions:")
        for key, value in sorted(result.items()):
            print(f"{key}: {value}")
    
    return result
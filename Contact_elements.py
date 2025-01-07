import opensim
import numpy as np
from importlib import import_module
from position_contact import calculate_contact_positions
import os

def create_contact_elements(model, model_path, heel_shift=0.0, verbose=False):
    """
    Creates contact elements for the OpenSim model using calculated positions and saves as a new model.
    
    Args:
        model: OpenSim model object
        model_path: Path to the original model file
        heel_shift: Shift value for heel position calculations (default: 0.0)
        verbose: Whether to print detailed information (default: False)
        
    Returns:
        str: Path to the saved contact model file
    """
    if verbose:
        print("Creating contact elements...")
    
    # Get body names
    calcn_r_name = 'calcn_r'
    calcn_l_name = 'calcn_l'
    toes_r_name = 'toes_r'
    toes_l_name = 'toes_l'
    hand_r_name = 'hand_r'
    hand_l_name = 'hand_l'
    
    # Calculate sphere positions
    positions = calculate_contact_positions(model, heel_shift, verbose)
    
    # Set sphere radius
    sphere_radius = 0.02  # Default radius for foot spheres
    
    # Create contact spheres for feet
    sphere_configs = [
        ('Sphere_Foot_1_R', calcn_r_name, positions['pos_sp1r']),
        ('Sphere_Foot_1_L', calcn_l_name, positions['pos_sp1l']),
        ('Sphere_Foot_2_R', calcn_r_name, positions['pos_sp2r']),
        ('Sphere_Foot_2_L', calcn_l_name, positions['pos_sp2l']),
        ('Sphere_Foot_3_R', calcn_r_name, positions['pos_sp3r']),
        ('Sphere_Foot_3_L', calcn_l_name, positions['pos_sp3l']),
        ('Sphere_Foot_4_R', calcn_r_name, positions['pos_sp4r']),
        ('Sphere_Foot_4_L', calcn_l_name, positions['pos_sp4l']),
        ('Sphere_Foot_5_R', toes_r_name, positions['pos_sp5r']),
        ('Sphere_Foot_5_L', toes_l_name, positions['pos_sp5l']),
        ('Sphere_Foot_6_R', toes_r_name, positions['pos_sp6r']),
        ('Sphere_Foot_6_L', toes_l_name, positions['pos_sp6l']),
        ('Sphere_Foot_7_R', calcn_r_name, positions['pos_sp7r']),
        ('Sphere_Foot_7_L', calcn_l_name, positions['pos_sp7l']),
        ('Sphere_Foot_8_R', calcn_r_name, positions['pos_sp8r']),
        ('Sphere_Foot_8_L', calcn_l_name, positions['pos_sp8l']),
        ('Sphere_Foot_9_R', calcn_r_name, positions['pos_sp9r']),
        ('Sphere_Foot_9_L', calcn_l_name, positions['pos_sp9l']),
        ('Sphere_Foot_10_R', calcn_r_name, positions['pos_sp10r']),
        ('Sphere_Foot_10_L', calcn_l_name, positions['pos_sp10l']),
        ('Sphere_Foot_11_R', calcn_r_name, positions['pos_sp11r']),
        ('Sphere_Foot_11_L', calcn_l_name, positions['pos_sp11l']),
        ('Sphere_Foot_12_R', calcn_r_name, positions['pos_sp12r']),
        ('Sphere_Foot_12_L', calcn_l_name, positions['pos_sp12l']),
        ('Sphere_Foot_13_R', toes_r_name, positions['pos_sp13r']),
        ('Sphere_Foot_13_L', toes_l_name, positions['pos_sp13l']),
        ('Sphere_Foot_14_R', toes_r_name, positions['pos_sp14r']),
        ('Sphere_Foot_14_L', toes_l_name, positions['pos_sp14l']),
    ]
    
    # Create and add all foot spheres
    for name, body_name, location in sphere_configs:
        sphere = opensim.ContactSphere()
        sphere.setName(name)
        body = model.getBodySet().get(body_name)
        frame = opensim.PhysicalFrame.safeDownCast(body)
        sphere.connectSocket_frame(frame)
        sphere.setLocation(opensim.Vec3(location[0], location[1], location[2]))
        sphere.setRadius(sphere_radius)
        model.addContactGeometry(sphere)
        
        if verbose:
            print(f"Created {name} on {body_name}")
    
    # Create hand contact spheres
    hand_sphere_configs = [
        ('Sphere_Hand_R', hand_r_name),
        ('Sphere_Hand_L', hand_l_name)
    ]
    
    # Create and add hand spheres
    for name, body_name in hand_sphere_configs:
        sphere = opensim.ContactSphere()
        sphere.setName(name)
        body = model.getBodySet().get(body_name)
        frame = opensim.PhysicalFrame.safeDownCast(body)
        sphere.connectSocket_frame(frame)
        sphere.setLocation(body.get_mass_center())
        sphere.setRadius(0.08)  # Specific radius for hand spheres
        model.addContactGeometry(sphere)
        
        if verbose:
            print(f"Created {name} on {body_name}")
    
    # Create ground contact half spaces
    ground_configs = [
        ('ground_Foot_L', 'Ground_L_Foot_Height'),
        ('ground_Foot_R', 'Ground_R_Foot_Height'),
        ('ground_Hand_R', 'Ground_R_Hand_Height'),
        ('ground_Hand_L', 'Ground_L_Hand_Height')
    ]
    
    # Create and add ground contact geometries
    for name, height_var in ground_configs:
        ground = opensim.ContactHalfSpace()
        ground.setName(name)
        ground_frame = model.getGround()
        ground.connectSocket_frame(ground_frame)
        # Get height from the position_contact module's global variables
        height = getattr(import_module('position_contact'), height_var)
        ground.setLocation(opensim.Vec3(0, height, 0))
        ground.setOrientation(opensim.Vec3(0, 0, -1.5708))
        model.addContactGeometry(ground)
        
        if verbose:
            print(f"Created {name} contact half space")
    
    # Finalize connections before saving
    model.finalizeConnections()
    
    # Save the model with contact elements
    base_path = os.path.splitext(model_path)[0]
    contact_model_path = f"{base_path}_contact.osim"
    model.printToXML(contact_model_path)
    
    if verbose:
        print(f"Saved contact model to: {contact_model_path}")
    
    return contact_model_path
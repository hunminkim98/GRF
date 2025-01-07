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
    
    # Create Hunt-Crossley force elements for each contact pair
    sphere_stiffness = 1E7 # Default stiffness value, 10000000

    # Hunt-Crossley forces for foot spheres
    ForceGround_Foot_1_R = opensim.HuntCrossleyForce()
    ForceGround_Foot_1_R.setName('ForceGround_Foot_1_R')
    ForceGround_Foot_1_R.set_appliesForce(True)
    ForceGround_Foot_1_R.addGeometry('ground_Foot_R Sphere_Foot_1_R')
    ForceGround_Foot_1_R.setStiffness(sphere_stiffness)
    ForceGround_Foot_1_R.setDissipation(0)
    ForceGround_Foot_1_R.setStaticFriction(0)
    ForceGround_Foot_1_R.setDynamicFriction(0)
    ForceGround_Foot_1_R.setViscousFriction(0)
    ForceGround_Foot_1_R.setTransitionVelocity(0.13)

    ForceGround_Foot_2_R = opensim.HuntCrossleyForce()
    ForceGround_Foot_2_R.setName('ForceGround_Foot_2_R')
    ForceGround_Foot_2_R.set_appliesForce(True)
    ForceGround_Foot_2_R.addGeometry('ground_Foot_R Sphere_Foot_2_R')
    ForceGround_Foot_2_R.setStiffness(sphere_stiffness)
    ForceGround_Foot_2_R.setDissipation(0)
    ForceGround_Foot_2_R.setStaticFriction(0)
    ForceGround_Foot_2_R.setDynamicFriction(0)
    ForceGround_Foot_2_R.setViscousFriction(0)
    ForceGround_Foot_2_R.setTransitionVelocity(0.13)

    ForceGround_Foot_1_L = opensim.HuntCrossleyForce()
    ForceGround_Foot_1_L.setName('ForceGround_Foot_1_L')
    ForceGround_Foot_1_L.set_appliesForce(True)
    ForceGround_Foot_1_L.addGeometry('ground_Foot_L Sphere_Foot_1_L')
    ForceGround_Foot_1_L.setStiffness(sphere_stiffness)
    ForceGround_Foot_1_L.setDissipation(0)
    ForceGround_Foot_1_L.setStaticFriction(0)
    ForceGround_Foot_1_L.setDynamicFriction(0)
    ForceGround_Foot_1_L.setViscousFriction(0)
    ForceGround_Foot_1_L.setTransitionVelocity(0.13)

    ForceGround_Foot_2_L = opensim.HuntCrossleyForce()
    ForceGround_Foot_2_L.setName('ForceGround_Foot_2_L')
    ForceGround_Foot_2_L.set_appliesForce(True)
    ForceGround_Foot_2_L.addGeometry('ground_Foot_L Sphere_Foot_2_L')
    ForceGround_Foot_2_L.setStiffness(sphere_stiffness)
    ForceGround_Foot_2_L.setDissipation(0)
    ForceGround_Foot_2_L.setStaticFriction(0)
    ForceGround_Foot_2_L.setDynamicFriction(0)
    ForceGround_Foot_2_L.setViscousFriction(0)
    ForceGround_Foot_2_L.setTransitionVelocity(0.13)

    ForceGround_Foot_3_R = opensim.HuntCrossleyForce()
    ForceGround_Foot_3_R.setName('ForceGround_Foot_3_R')
    ForceGround_Foot_3_R.set_appliesForce(True)
    ForceGround_Foot_3_R.addGeometry('ground_Foot_R Sphere_Foot_3_R')
    ForceGround_Foot_3_R.setStiffness(sphere_stiffness)
    ForceGround_Foot_3_R.setDissipation(0)
    ForceGround_Foot_3_R.setStaticFriction(0)
    ForceGround_Foot_3_R.setDynamicFriction(0)
    ForceGround_Foot_3_R.setViscousFriction(0)
    ForceGround_Foot_3_R.setTransitionVelocity(0.13)

    ForceGround_Foot_3_L = opensim.HuntCrossleyForce()
    ForceGround_Foot_3_L.setName('ForceGround_Foot_3_L')
    ForceGround_Foot_3_L.set_appliesForce(True)
    ForceGround_Foot_3_L.addGeometry('ground_Foot_L Sphere_Foot_3_L')
    ForceGround_Foot_3_L.setStiffness(sphere_stiffness)
    ForceGround_Foot_3_L.setDissipation(0)
    ForceGround_Foot_3_L.setStaticFriction(0)
    ForceGround_Foot_3_L.setDynamicFriction(0)
    ForceGround_Foot_3_L.setViscousFriction(0)
    ForceGround_Foot_3_L.setTransitionVelocity(0.13)

    ForceGround_Foot_4_R = opensim.HuntCrossleyForce()
    ForceGround_Foot_4_R.setName('ForceGround_Foot_4_R')
    ForceGround_Foot_4_R.set_appliesForce(True)
    ForceGround_Foot_4_R.addGeometry('ground_Foot_R Sphere_Foot_4_R')
    ForceGround_Foot_4_R.setStiffness(sphere_stiffness)
    ForceGround_Foot_4_R.setDissipation(0)
    ForceGround_Foot_4_R.setStaticFriction(0)
    ForceGround_Foot_4_R.setDynamicFriction(0)
    ForceGround_Foot_4_R.setViscousFriction(0)
    ForceGround_Foot_4_R.setTransitionVelocity(0.13)

    ForceGround_Foot_4_L = opensim.HuntCrossleyForce()
    ForceGround_Foot_4_L.setName('ForceGround_Foot_4_L')
    ForceGround_Foot_4_L.set_appliesForce(True)
    ForceGround_Foot_4_L.addGeometry('ground_Foot_L Sphere_Foot_4_L')
    ForceGround_Foot_4_L.setStiffness(sphere_stiffness)
    ForceGround_Foot_4_L.setDissipation(0)
    ForceGround_Foot_4_L.setStaticFriction(0)
    ForceGround_Foot_4_L.setDynamicFriction(0)
    ForceGround_Foot_4_L.setViscousFriction(0)
    ForceGround_Foot_4_L.setTransitionVelocity(0.13)

    ForceGround_Foot_5_R = opensim.HuntCrossleyForce()
    ForceGround_Foot_5_R.setName('ForceGround_Foot_5_R')
    ForceGround_Foot_5_R.set_appliesForce(True)
    ForceGround_Foot_5_R.addGeometry('ground_Foot_R Sphere_Foot_5_R')
    ForceGround_Foot_5_R.setStiffness(sphere_stiffness)
    ForceGround_Foot_5_R.setDissipation(0)
    ForceGround_Foot_5_R.setStaticFriction(0)
    ForceGround_Foot_5_R.setDynamicFriction(0)
    ForceGround_Foot_5_R.setViscousFriction(0)
    ForceGround_Foot_5_R.setTransitionVelocity(0.13)

    ForceGround_Foot_5_L = opensim.HuntCrossleyForce()
    ForceGround_Foot_5_L.setName('ForceGround_Foot_5_L')
    ForceGround_Foot_5_L.set_appliesForce(True)
    ForceGround_Foot_5_L.addGeometry('ground_Foot_L Sphere_Foot_5_L')
    ForceGround_Foot_5_L.setStiffness(sphere_stiffness)
    ForceGround_Foot_5_L.setDissipation(0)
    ForceGround_Foot_5_L.setStaticFriction(0)
    ForceGround_Foot_5_L.setDynamicFriction(0)
    ForceGround_Foot_5_L.setViscousFriction(0)
    ForceGround_Foot_5_L.setTransitionVelocity(0.13)

    ForceGround_Foot_6_R = opensim.HuntCrossleyForce()
    ForceGround_Foot_6_R.setName('ForceGround_Foot_6_R')
    ForceGround_Foot_6_R.set_appliesForce(True)
    ForceGround_Foot_6_R.addGeometry('ground_Foot_R Sphere_Foot_6_R')
    ForceGround_Foot_6_R.setStiffness(sphere_stiffness)
    ForceGround_Foot_6_R.setDissipation(0)
    ForceGround_Foot_6_R.setStaticFriction(0)
    ForceGround_Foot_6_R.setDynamicFriction(0)
    ForceGround_Foot_6_R.setViscousFriction(0)
    ForceGround_Foot_6_R.setTransitionVelocity(0.13)

    ForceGround_Foot_6_L = opensim.HuntCrossleyForce()
    ForceGround_Foot_6_L.setName('ForceGround_Foot_6_L')
    ForceGround_Foot_6_L.set_appliesForce(True)
    ForceGround_Foot_6_L.addGeometry('ground_Foot_L Sphere_Foot_6_L')
    ForceGround_Foot_6_L.setStiffness(sphere_stiffness)
    ForceGround_Foot_6_L.setDissipation(0)
    ForceGround_Foot_6_L.setStaticFriction(0)
    ForceGround_Foot_6_L.setDynamicFriction(0)
    ForceGround_Foot_6_L.setViscousFriction(0)
    ForceGround_Foot_6_L.setTransitionVelocity(0.13)

    ForceGround_Foot_7_R = opensim.HuntCrossleyForce()
    ForceGround_Foot_7_R.setName('ForceGround_Foot_7_R')
    ForceGround_Foot_7_R.set_appliesForce(True)
    ForceGround_Foot_7_R.addGeometry('ground_Foot_R Sphere_Foot_7_R')
    ForceGround_Foot_7_R.setStiffness(sphere_stiffness)
    ForceGround_Foot_7_R.setDissipation(0)
    ForceGround_Foot_7_R.setStaticFriction(0)
    ForceGround_Foot_7_R.setDynamicFriction(0)
    ForceGround_Foot_7_R.setViscousFriction(0)
    ForceGround_Foot_7_R.setTransitionVelocity(0.13)

    ForceGround_Foot_7_L = opensim.HuntCrossleyForce()
    ForceGround_Foot_7_L.setName('ForceGround_Foot_7_L')
    ForceGround_Foot_7_L.set_appliesForce(True)
    ForceGround_Foot_7_L.addGeometry('ground_Foot_L Sphere_Foot_7_L')
    ForceGround_Foot_7_L.setStiffness(sphere_stiffness)
    ForceGround_Foot_7_L.setDissipation(0)
    ForceGround_Foot_7_L.setStaticFriction(0)
    ForceGround_Foot_7_L.setDynamicFriction(0)
    ForceGround_Foot_7_L.setViscousFriction(0)
    ForceGround_Foot_7_L.setTransitionVelocity(0.13)

    ForceGround_Foot_8_R = opensim.HuntCrossleyForce()
    ForceGround_Foot_8_R.setName('ForceGround_Foot_8_R')
    ForceGround_Foot_8_R.set_appliesForce(True)
    ForceGround_Foot_8_R.addGeometry('ground_Foot_R Sphere_Foot_8_R')
    ForceGround_Foot_8_R.setStiffness(sphere_stiffness)
    ForceGround_Foot_8_R.setDissipation(0)
    ForceGround_Foot_8_R.setStaticFriction(0)
    ForceGround_Foot_8_R.setDynamicFriction(0)
    ForceGround_Foot_8_R.setViscousFriction(0)
    ForceGround_Foot_8_R.setTransitionVelocity(0.13)

    ForceGround_Foot_8_L = opensim.HuntCrossleyForce()
    ForceGround_Foot_8_L.setName('ForceGround_Foot_8_L')
    ForceGround_Foot_8_L.set_appliesForce(True)
    ForceGround_Foot_8_L.addGeometry('ground_Foot_L Sphere_Foot_8_L')
    ForceGround_Foot_8_L.setStiffness(sphere_stiffness)
    ForceGround_Foot_8_L.setDissipation(0)
    ForceGround_Foot_8_L.setStaticFriction(0)
    ForceGround_Foot_8_L.setDynamicFriction(0)
    ForceGround_Foot_8_L.setViscousFriction(0)
    ForceGround_Foot_8_L.setTransitionVelocity(0.13)

    ForceGround_Foot_9_R = opensim.HuntCrossleyForce()
    ForceGround_Foot_9_R.setName('ForceGround_Foot_9_R')
    ForceGround_Foot_9_R.set_appliesForce(True)
    ForceGround_Foot_9_R.addGeometry('ground_Foot_R Sphere_Foot_9_R')
    ForceGround_Foot_9_R.setStiffness(sphere_stiffness)
    ForceGround_Foot_9_R.setDissipation(0)
    ForceGround_Foot_9_R.setStaticFriction(0)
    ForceGround_Foot_9_R.setDynamicFriction(0)
    ForceGround_Foot_9_R.setViscousFriction(0)
    ForceGround_Foot_9_R.setTransitionVelocity(0.13)

    ForceGround_Foot_9_L = opensim.HuntCrossleyForce()
    ForceGround_Foot_9_L.setName('ForceGround_Foot_9_L')
    ForceGround_Foot_9_L.set_appliesForce(True)
    ForceGround_Foot_9_L.addGeometry('ground_Foot_L Sphere_Foot_9_L')
    ForceGround_Foot_9_L.setStiffness(sphere_stiffness)
    ForceGround_Foot_9_L.setDissipation(0)
    ForceGround_Foot_9_L.setStaticFriction(0)
    ForceGround_Foot_9_L.setDynamicFriction(0)
    ForceGround_Foot_9_L.setViscousFriction(0)
    ForceGround_Foot_9_L.setTransitionVelocity(0.13)

    ForceGround_Foot_10_R = opensim.HuntCrossleyForce()
    ForceGround_Foot_10_R.setName('ForceGround_Foot_10_R')
    ForceGround_Foot_10_R.set_appliesForce(True)
    ForceGround_Foot_10_R.addGeometry('ground_Foot_R Sphere_Foot_10_R')
    ForceGround_Foot_10_R.setStiffness(sphere_stiffness)
    ForceGround_Foot_10_R.setDissipation(0)
    ForceGround_Foot_10_R.setStaticFriction(0)
    ForceGround_Foot_10_R.setDynamicFriction(0)
    ForceGround_Foot_10_R.setViscousFriction(0)
    ForceGround_Foot_10_R.setTransitionVelocity(0.13)

    ForceGround_Foot_10_L = opensim.HuntCrossleyForce()
    ForceGround_Foot_10_L.setName('ForceGround_Foot_10_L')
    ForceGround_Foot_10_L.set_appliesForce(True)
    ForceGround_Foot_10_L.addGeometry('ground_Foot_L Sphere_Foot_10_L')
    ForceGround_Foot_10_L.setStiffness(sphere_stiffness)
    ForceGround_Foot_10_L.setDissipation(0)
    ForceGround_Foot_10_L.setStaticFriction(0)
    ForceGround_Foot_10_L.setDynamicFriction(0)
    ForceGround_Foot_10_L.setViscousFriction(0)
    ForceGround_Foot_10_L.setTransitionVelocity(0.13)

    ForceGround_Foot_11_R = opensim.HuntCrossleyForce()
    ForceGround_Foot_11_R.setName('ForceGround_Foot_11_R')
    ForceGround_Foot_11_R.set_appliesForce(True)
    ForceGround_Foot_11_R.addGeometry('ground_Foot_R Sphere_Foot_11_R')
    ForceGround_Foot_11_R.setStiffness(sphere_stiffness)
    ForceGround_Foot_11_R.setDissipation(0)
    ForceGround_Foot_11_R.setStaticFriction(0)
    ForceGround_Foot_11_R.setDynamicFriction(0)
    ForceGround_Foot_11_R.setViscousFriction(0)
    ForceGround_Foot_11_R.setTransitionVelocity(0.13)

    ForceGround_Foot_11_L = opensim.HuntCrossleyForce()
    ForceGround_Foot_11_L.setName('ForceGround_Foot_11_L')
    ForceGround_Foot_11_L.set_appliesForce(True)
    ForceGround_Foot_11_L.addGeometry('ground_Foot_L Sphere_Foot_11_L')
    ForceGround_Foot_11_L.setStiffness(sphere_stiffness)
    ForceGround_Foot_11_L.setDissipation(0)
    ForceGround_Foot_11_L.setStaticFriction(0)
    ForceGround_Foot_11_L.setDynamicFriction(0)
    ForceGround_Foot_11_L.setViscousFriction(0)
    ForceGround_Foot_11_L.setTransitionVelocity(0.13)

    ForceGround_Foot_12_R = opensim.HuntCrossleyForce()
    ForceGround_Foot_12_R.setName('ForceGround_Foot_12_R')
    ForceGround_Foot_12_R.set_appliesForce(True)
    ForceGround_Foot_12_R.addGeometry('ground_Foot_R Sphere_Foot_12_R')
    ForceGround_Foot_12_R.setStiffness(sphere_stiffness)
    ForceGround_Foot_12_R.setDissipation(0)
    ForceGround_Foot_12_R.setStaticFriction(0)
    ForceGround_Foot_12_R.setDynamicFriction(0)
    ForceGround_Foot_12_R.setViscousFriction(0)
    ForceGround_Foot_12_R.setTransitionVelocity(0.13)

    ForceGround_Foot_12_L = opensim.HuntCrossleyForce()
    ForceGround_Foot_12_L.setName('ForceGround_Foot_12_L')
    ForceGround_Foot_12_L.set_appliesForce(True)
    ForceGround_Foot_12_L.addGeometry('ground_Foot_L Sphere_Foot_12_L')
    ForceGround_Foot_12_L.setStiffness(sphere_stiffness)
    ForceGround_Foot_12_L.setDissipation(0)
    ForceGround_Foot_12_L.setStaticFriction(0)
    ForceGround_Foot_12_L.setDynamicFriction(0)
    ForceGround_Foot_12_L.setViscousFriction(0)
    ForceGround_Foot_12_L.setTransitionVelocity(0.13)

    ForceGround_Foot_13_R = opensim.HuntCrossleyForce()
    ForceGround_Foot_13_R.setName('ForceGround_Foot_13_R')
    ForceGround_Foot_13_R.set_appliesForce(True)
    ForceGround_Foot_13_R.addGeometry('ground_Foot_R Sphere_Foot_13_R')
    ForceGround_Foot_13_R.setStiffness(sphere_stiffness)
    ForceGround_Foot_13_R.setDissipation(0)
    ForceGround_Foot_13_R.setStaticFriction(0)
    ForceGround_Foot_13_R.setDynamicFriction(0)
    ForceGround_Foot_13_R.setViscousFriction(0)
    ForceGround_Foot_13_R.setTransitionVelocity(0.13)

    ForceGround_Foot_13_L = opensim.HuntCrossleyForce()
    ForceGround_Foot_13_L.setName('ForceGround_Foot_13_L')
    ForceGround_Foot_13_L.set_appliesForce(True)
    ForceGround_Foot_13_L.addGeometry('ground_Foot_L Sphere_Foot_13_L')
    ForceGround_Foot_13_L.setStiffness(sphere_stiffness)
    ForceGround_Foot_13_L.setDissipation(0)
    ForceGround_Foot_13_L.setStaticFriction(0)
    ForceGround_Foot_13_L.setDynamicFriction(0)
    ForceGround_Foot_13_L.setViscousFriction(0)
    ForceGround_Foot_13_L.setTransitionVelocity(0.13)

    ForceGround_Foot_14_R = opensim.HuntCrossleyForce()
    ForceGround_Foot_14_R.setName('ForceGround_Foot_14_R')
    ForceGround_Foot_14_R.set_appliesForce(True)
    ForceGround_Foot_14_R.addGeometry('ground_Foot_R Sphere_Foot_14_R')
    ForceGround_Foot_14_R.setStiffness(sphere_stiffness)
    ForceGround_Foot_14_R.setDissipation(0)
    ForceGround_Foot_14_R.setStaticFriction(0)
    ForceGround_Foot_14_R.setDynamicFriction(0)
    ForceGround_Foot_14_R.setViscousFriction(0)
    ForceGround_Foot_14_R.setTransitionVelocity(0.13)

    ForceGround_Foot_14_L = opensim.HuntCrossleyForce()
    ForceGround_Foot_14_L.setName('ForceGround_Foot_14_L')
    ForceGround_Foot_14_L.set_appliesForce(True)
    ForceGround_Foot_14_L.addGeometry('ground_Foot_L Sphere_Foot_14_L')
    ForceGround_Foot_14_L.setStiffness(sphere_stiffness)
    ForceGround_Foot_14_L.setDissipation(0)
    ForceGround_Foot_14_L.setStaticFriction(0)
    ForceGround_Foot_14_L.setDynamicFriction(0)
    ForceGround_Foot_14_L.setViscousFriction(0)
    ForceGround_Foot_14_L.setTransitionVelocity(0.13)

    # HuntCrossley forces for hands
    ForceGround_Hand_L = opensim.HuntCrossleyForce()
    ForceGround_Hand_L.setName('ForceGround_Hand_L')
    ForceGround_Hand_L.set_appliesForce(True)
    ForceGround_Hand_L.addGeometry('ground_Hand_L Sphere_Hand_L')
    ForceGround_Hand_L.setStiffness(sphere_stiffness)
    ForceGround_Hand_L.setDissipation(0)
    ForceGround_Hand_L.setStaticFriction(0)
    ForceGround_Hand_L.setDynamicFriction(0)
    ForceGround_Hand_L.setViscousFriction(0)
    ForceGround_Hand_L.setTransitionVelocity(0.13)

    ForceGround_Hand_R = opensim.HuntCrossleyForce()
    ForceGround_Hand_R.setName('ForceGround_Hand_R')
    ForceGround_Hand_R.set_appliesForce(True)
    ForceGround_Hand_R.addGeometry('ground_Hand_R Sphere_Hand_R')
    ForceGround_Hand_R.setStiffness(sphere_stiffness)
    ForceGround_Hand_R.setDissipation(0)
    ForceGround_Hand_R.setStaticFriction(0)
    ForceGround_Hand_R.setDynamicFriction(0)
    ForceGround_Hand_R.setViscousFriction(0)
    ForceGround_Hand_R.setTransitionVelocity(0.13)

    # Add foot forces to model
    model.getForceSet().cloneAndAppend(ForceGround_Foot_1_R)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_2_R)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_1_L)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_2_L)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_3_R)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_3_L)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_4_R)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_4_L)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_5_R)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_5_L)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_6_R)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_6_L)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_7_R)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_7_L)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_8_R)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_8_L)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_9_R)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_9_L)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_10_R)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_10_L)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_11_R)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_11_L)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_12_R)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_12_L)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_13_R)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_13_L)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_14_R)
    model.getForceSet().cloneAndAppend(ForceGround_Foot_14_L)

    # Add hand forces to model
    model.getForceSet().cloneAndAppend(ForceGround_Hand_R)
    model.getForceSet().cloneAndAppend(ForceGround_Hand_L)

    # Finalize connections
    model.finalizeConnections()

    if verbose:
        print("Added Hunt-Crossley contact forces to the model")

    # Print model
    model_name = os.path.splitext(os.path.basename(model_path))[0]
    contact_model_path = os.path.join(os.path.dirname(model_path), f'{model_name}_contact.osim')
    model.printToXML(contact_model_path)
    
    if verbose:
        print(f"Saved contact model to: {contact_model_path}")
        
    return contact_model_path
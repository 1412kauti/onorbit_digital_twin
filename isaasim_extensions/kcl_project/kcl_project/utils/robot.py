# kcl_project/utils/robot.py

import omni.usd
import carb
from pxr import UsdGeom
import os

def load_usd_robot_simple(stage, usd_path, prim_path="/World/Robot",
                         position=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)):
    """
    Simple robot loading that preserves joint functionality.
    """
    try:
        # Remove existing prim if it exists
        if stage.GetPrimAtPath(prim_path):
            stage.RemovePrim(prim_path)

        # Create simple Xform and add reference directly
        from pxr import UsdGeom
        robot_xform = UsdGeom.Xform.Define(stage, prim_path)

        # Apply transforms
        robot_xform.AddTranslateOp().Set(position)
        robot_xform.AddRotateXYZOp().Set(rotation)
        robot_xform.AddScaleOp().Set(scale)

        # Add reference directly to the Xform
        references = robot_xform.GetPrim().GetReferences()
        references.AddReference(usd_path)

        carb.log_info(f"[Robot] Loaded robot with simple reference: {prim_path}")
        return True, prim_path

    except Exception as e:
        carb.log_error(f"[Robot] Error loading robot: {str(e)}")
        return False, None

def _strip_nested_articulation_roots(prim):
    """
    FIXED: Remove nested articulation roots to prevent conflicts.
    """
    from pxr import UsdPhysics
    
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        carb.log_info(f"[Robot] Removed nested ArticulationRootAPI from {prim.GetPath()}")
    
    for child in prim.GetChildren():
        _strip_nested_articulation_roots(child)

def configure_robot_joints_robust(stage, robot_prim_path):
    """
    FIXED: Robust robot joint configuration with deferred initialization.
    """
    try:
        import omni.timeline
        
        # Step 1: Verify robot prim exists
        robot_prim = stage.GetPrimAtPath(robot_prim_path)
        if not robot_prim or not robot_prim.IsValid():
            carb.log_error(f"[Robot] Robot prim not found: {robot_prim_path}")
            return None

        # Step 2: FIXED - Strip nested articulation roots first
        _strip_nested_articulation_roots(robot_prim)

        # Step 3: Apply ArticulationRootAPI to main robot prim
        from pxr import UsdPhysics
        if not robot_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
            carb.log_info("[Robot] Applied ArticulationRootAPI to robot")

        # Step 4: FIXED - Defer articulation creation until timeline plays
        timeline = omni.timeline.get_timeline_interface()
        
        def _init_articulation():
            """Initialize articulation when timeline is playing."""
            try:
                from omni.isaac.core.articulations import Articulation
                
                robot = Articulation(prim_path=robot_prim_path, name="canadarm2")
                robot.initialize()
                
                if robot.num_dof > 0:
                    carb.log_info(f"[Robot] Successfully created articulation with {robot.num_dof} DOF")
                    
                    # Configure joints
                    for i in range(robot.num_dof):
                        joint_name = robot.dof_names[i]
                        robot.set_joint_drive_parameters(
                            joint_index=i,
                            stiffness=1000.0,
                            damping=100.0,
                            max_force=5000.0
                        )
                        robot.set_joint_position_limits(i, -180.0, 180.0)
                        carb.log_info(f"[Robot] Configured joint {i}: {joint_name}")
                    
                    return robot
                else:
                    carb.log_warn("[Robot] Articulation created but has 0 DOF")
                    return None
                    
            except Exception as e:
                carb.log_error(f"[Robot] Error during articulation setup: {str(e)}")
                return None

        # If timeline is not playing, defer initialization
        if not timeline.is_playing():
            def _on_play(event):
                if event.type == int(omni.timeline.TimelineEventType.PLAY):
                    timeline_event_stream.unsubscribe()
                    _init_articulation()
            
            timeline_event_stream = timeline.get_timeline_event_stream()
            timeline_event_stream.create_subscription_to_pop(_on_play)
            carb.log_info("[Robot] Articulation initialization deferred until timeline plays")
            return None  # Will initialize later
        else:
            return _init_articulation()

    except Exception as e:
        carb.log_error(f"[Robot] Error in robust joint configuration: {str(e)}")
        return None

def configure_robot_joints_usd_direct(stage, robot_prim_path):
    """
    Configure robot joints using direct USD APIs as fallback.
    """
    try:
        from pxr import UsdPhysics
        
        robot_prim = stage.GetPrimAtPath(robot_prim_path)
        if not robot_prim:
            return False

        # FIXED: Strip nested articulation roots first
        _strip_nested_articulation_roots(robot_prim)

        # Ensure robot has articulation root
        if not robot_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            UsdPhysics.ArticulationRootAPI.Apply(robot_prim)

        # Find and configure all joints
        joint_count = 0
        
        def configure_joints_recursive(prim):
            nonlocal joint_count
            
            # Check if this prim is a joint
            if prim.GetTypeName() in ["PhysicsRevoluteJoint", "PhysicsPrismaticJoint"]:
                joint_name = prim.GetName()
                
                # Apply drive API if not present
                if not prim.HasAPI(UsdPhysics.DriveAPI):
                    drive_api = UsdPhysics.DriveAPI.Apply(prim, "drive")
                else:
                    drive_api = UsdPhysics.DriveAPI(prim, "drive")

                # Set drive parameters
                drive_api.CreateStiffnessAttr().Set(1000.0)
                drive_api.CreateDampingAttr().Set(100.0)
                drive_api.CreateMaxForceAttr().Set(5000.0)
                
                joint_count += 1
                carb.log_info(f"[Robot] Configured USD joint: {joint_name}")

            # Recursively process children
            for child in prim.GetChildren():
                configure_joints_recursive(child)

        configure_joints_recursive(robot_prim)
        carb.log_info(f"[Robot] Configured {joint_count} joints via USD")
        return joint_count > 0

    except Exception as e:
        carb.log_error(f"[Robot] Error in USD joint configuration: {str(e)}")
        return False

def load_usd_robot(stage, usd_path, prim_path="/World/Robot",
                  position=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1),
                  max_force=10000, damping=5100, stiffness=1500):
    """
    Load a USD robot file and configure joint parameters.
    """
    try:
        # Remove existing prim at the path if it exists
        if stage.GetPrimAtPath(prim_path):
            stage.RemovePrim(prim_path)

        # Create an Xform prim to hold the reference
        xform = UsdGeom.Xform.Define(stage, prim_path)

        # Set transform
        xform.AddTranslateOp().Set(position)
        xform.AddRotateXYZOp().Set(rotation)
        xform.AddScaleOp().Set(scale)

        # Add the USD file as a reference
        references = xform.GetPrim().GetReferences()
        references.AddReference(usd_path)

        # Configure joint parameters after loading
        configure_joint_parameters(stage, prim_path, max_force, damping, stiffness)

        carb.log_info(f"[Robot] Successfully loaded USD robot: {usd_path} at {prim_path}")
        return True, prim_path

    except Exception as e:
        carb.log_error(f"[Robot] Error loading USD robot: {str(e)}")
        return False, None

def configure_joint_parameters(stage, robot_prim_path, max_force, damping, stiffness):
    """
    Configure joint parameters for all joints in the robot.
    """
    try:
        from pxr import UsdPhysics

        # Get the robot prim
        robot_prim = stage.GetPrimAtPath(robot_prim_path)
        if not robot_prim:
            carb.log_error(f"[Robot] Robot prim not found at {robot_prim_path}")
            return

        # Find all joint prims recursively
        def find_joints(prim):
            joints = []
            # Check if prim is a joint by type name
            type_name = prim.GetTypeName()
            if type_name in ["PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "PhysicsSphericalJoint"]:
                joints.append(prim)
            for child in prim.GetChildren():
                joints.extend(find_joints(child))
            return joints

        joints = find_joints(robot_prim)
        carb.log_info(f"[Robot] Found {len(joints)} joints to configure")

        # Configure each joint
        for joint_prim in joints:
            joint_name = joint_prim.GetName()
            
            drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "drive")
            if not drive_api:
                drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "drive")

            # Set the joint parameters
            drive_api.CreateMaxForceAttr().Set(max_force)
            drive_api.CreateDampingAttr().Set(damping)
            drive_api.CreateStiffnessAttr().Set(stiffness)

            carb.log_info(f"[Robot] Configured joint: {joint_name}")

    except Exception as e:
        carb.log_error(f"[Robot] Error configuring joint parameters: {str(e)}")

def remove_robot(stage, prim_path):
    """
    Remove a robot from the stage.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if prim and prim.IsValid():
        stage.RemovePrim(prim_path)
        carb.log_info(f"[Robot] Removed robot at {prim_path}")
        return True
    return False

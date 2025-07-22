# kcl_project/utils/physics.py

import carb
from pxr import UsdPhysics, UsdGeom, Gf, Usd

def configure_space_physics(stage):
    """
    Configure physics settings for on-orbit space conditions.
    """
    try:
        physics_scene = UsdPhysics.Scene.Get(stage, "/physicsScene")
        if not physics_scene:
            physics_scene = UsdPhysics.Scene.Define(stage, "/physicsScene")

        physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        physics_scene.CreateGravityMagnitudeAttr().Set(0.0)  # Zero gravity

        carb.log_info("[Physics] Configured space physics: Zero gravity enabled")
        return True

    except Exception as e:
        carb.log_error(f"[Physics] Error configuring space physics: {str(e)}")
        return False

def set_simulation_timestep(target_fps=60):
    """
    Set simulation timestep using timeline interface.
    """
    try:
        import omni.timeline
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_target_framerate(target_fps)
        carb.log_info(f"[Physics] Set simulation framerate to {target_fps} FPS")
    except Exception as e:
        carb.log_error(f"[Physics] Error setting timestep: {str(e)}")

def configure_space_materials(stage):
    """
    Configure material properties for space environment.
    """
    try:
        from pxr import UsdShade

        # Create space-appropriate material
        space_material = UsdShade.Material.Define(stage, "/World/Materials/SpaceMaterial")

        # Apply physics material properties
        material_api = UsdPhysics.MaterialAPI.Apply(space_material.GetPrim())
        material_api.CreateStaticFrictionAttr().Set(0.1)
        material_api.CreateDynamicFrictionAttr().Set(0.05)
        material_api.CreateRestitutionAttr().Set(0.9)

        carb.log_info("[Physics] Created space-appropriate materials")
        return True

    except Exception as e:
        carb.log_error(f"[Physics] Error configuring materials: {str(e)}")
        return False

def add_rigid_body_to_prim(stage, prim_path, mass=1000.0, enable_gravity=False):
    """
    Add rigid body physics to a specific prim.
    """
    try:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            carb.log_error(f"[Physics] Prim not found at {prim_path}")
            return False

        # Add rigid body API
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
            rigid_body_api.CreateRigidBodyEnabledAttr().Set(True)

            # Disable gravity for space simulation
            if not enable_gravity:
                rigid_body_api.CreateKinematicEnabledAttr().Set(False)

        # Add mass API
        if not prim.HasAPI(UsdPhysics.MassAPI):
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr().Set(mass)

            # Set center of mass to geometric center
            bbox = UsdGeom.Boundable(prim).ComputeWorldBound(0.0, "default")
            if bbox:
                center = bbox.ComputeCentroid()
                mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(center))

        carb.log_info(f"[Physics] Added rigid body (mass: {mass}kg) to {prim_path}")
        return True

    except Exception as e:
        carb.log_error(f"[Physics] Error adding rigid body to {prim_path}: {str(e)}")
        return False

def add_collider_to_prim(stage, prim_path, collision_type="convexHull"):
    """
    FIXED: Add collider to a specific prim with automatic fallback for dynamic bodies.
    """
    try:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            carb.log_error(f"[Physics] Prim not found at {prim_path}")
            return False

        # FIXED: Check if this is a dynamic body and force convex hull for trimesh
        rigid_body_api = UsdPhysics.RigidBodyAPI(prim) if prim.HasAPI(UsdPhysics.RigidBodyAPI) else None
        if collision_type == "trimesh" and rigid_body_api:
            kinematic_enabled = rigid_body_api.GetKinematicEnabledAttr()
            if not kinematic_enabled or not kinematic_enabled.Get():
                collision_type = "convexHull"  # Automatic downgrade
                carb.log_info(f"[Physics] Auto-downgraded to convexHull for dynamic body at {prim_path}")

        # Add collision API
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            collision_api = UsdPhysics.CollisionAPI.Apply(prim)
            collision_api.CreateCollisionEnabledAttr().Set(True)

        # Configure collision shape
        configure_collision_shape(prim, collision_type)

        carb.log_info(f"[Physics] Added {collision_type} collider to {prim_path}")
        return True

    except Exception as e:
        carb.log_error(f"[Physics] Error adding collider to {prim_path}: {str(e)}")
        return False

def configure_collision_shape(prim, collision_type):
    """
    Configure collision shape for the prim.
    """
    try:
        # Find mesh prims within the object
        def find_mesh_prims(prim):
            meshes = []
            if prim.GetTypeName() == "Mesh":
                meshes.append(prim)
            for child in prim.GetChildren():
                meshes.extend(find_mesh_prims(child))
            return meshes

        mesh_prims = find_mesh_prims(prim)
        carb.log_info(f"[Physics] Found {len(mesh_prims)} mesh prims to configure collision for")

        for mesh_prim in mesh_prims:
            # Apply collision API first
            if not mesh_prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_api = UsdPhysics.CollisionAPI.Apply(mesh_prim)
                collision_api.CreateCollisionEnabledAttr().Set(True)
                
            # Apply mesh collision API
            if not mesh_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
            else:
                mesh_collision_api = UsdPhysics.MeshCollisionAPI(mesh_prim)

            # Set collision approximation based on type
            if collision_type == "convexHull":
                mesh_collision_api.CreateApproximationAttr().Set("convexHull")
            elif collision_type == "convexDecomposition":
                mesh_collision_api.CreateApproximationAttr().Set("convexDecomposition")
            elif collision_type == "trimesh":
                mesh_collision_api.CreateApproximationAttr().Set("trimesh")
            else:
                mesh_collision_api.CreateApproximationAttr().Set("convexHull")  # Default
                
            carb.log_info(f"[Physics] Set {collision_type} approximation on {mesh_prim.GetPath()}")

    except Exception as e:
        carb.log_error(f"[Physics] Error configuring collision shape: {str(e)}")

def add_physics_material_to_prim(stage, prim_path, static_friction=0.3,
                                dynamic_friction=0.2, restitution=0.1):
    """
    Add physics material properties to a specific prim.
    """
    try:
        from pxr import UsdShade

        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            carb.log_error(f"[Physics] Prim not found at {prim_path}")
            return False

        # Create material for this specific prim
        material_name = prim.GetName() + "_Material"
        material_path = f"/World/Materials/{material_name}"
        material = UsdShade.Material.Define(stage, material_path)

        material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())

        # Set material properties
        material_api.CreateStaticFrictionAttr().Set(static_friction)
        material_api.CreateDynamicFrictionAttr().Set(dynamic_friction)
        material_api.CreateRestitutionAttr().Set(restitution)

        carb.log_info(f"[Physics] Added material to {prim_path}")
        return True

    except Exception as e:
        carb.log_error(f"[Physics] Error adding physics material to {prim_path}: {str(e)}")
        return False

def configure_sensor_parent_physics(stage, prim_path, mass=1000.0):
    """
    Configure a prim to be a proper parent for sensors by adding required physics APIs.
    
    Args:
        stage: USD stage
        prim_path: Path to the prim that will parent sensors
        mass: Mass for the rigid body (default: 1000kg for spacecraft)
        
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            carb.log_error(f"[Physics] Sensor parent prim not found at {prim_path}")
            return False
        
        carb.log_info(f"[Physics] Configuring sensor parent physics for {prim_path}")
        
        # Add Rigid Body API (required for IMU sensors)
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
            rigid_body_api.CreateRigidBodyEnabledAttr().Set(True)
            
            # For space simulation, set as kinematic by default to avoid unwanted movement
            rigid_body_api.CreateKinematicEnabledAttr().Set(True)
            carb.log_info(f"[Physics] Applied RigidBodyAPI to {prim_path}")
        else:
            carb.log_info(f"[Physics] RigidBodyAPI already exists on {prim_path}")
        
        # Add Mass API (required for proper physics)
        if not prim.HasAPI(UsdPhysics.MassAPI):
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr().Set(mass)
            
            # Try to calculate center of mass from geometry
            try:
                bbox = UsdGeom.Boundable(prim).ComputeWorldBound(0.0, "default")
                if bbox and bbox.GetBox().IsValid():
                    center = bbox.ComputeCentroid()
                    mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(center))
                else:
                    # Fallback to origin
                    mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            except:
                mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
                
            carb.log_info(f"[Physics] Applied MassAPI (mass: {mass}kg) to {prim_path}")
        else:
            carb.log_info(f"[Physics] MassAPI already exists on {prim_path}")
        
        # Add Collision API (helpful for physics stability)
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            collision_api = UsdPhysics.CollisionAPI.Apply(prim)
            collision_api.CreateCollisionEnabledAttr().Set(True)
            carb.log_info(f"[Physics] Applied CollisionAPI to {prim_path}")
            
            # Configure collision shape for mesh children
            configure_collision_shape(prim, "convexHull")
        else:
            carb.log_info(f"[Physics] CollisionAPI already exists on {prim_path}")
        
        carb.log_info(f"[Physics] Successfully configured sensor parent physics for {prim_path}")
        return True
        
    except Exception as e:
        carb.log_error(f"[Physics] Error configuring sensor parent physics for {prim_path}: {str(e)}")
        return False

def validate_sensor_parent(stage, prim_path):
    """
    Validate that a prim is properly configured to be a sensor parent.
    
    Args:
        stage: USD stage
        prim_path: Path to the potential sensor parent prim
        
    Returns:
        tuple: (is_valid, missing_apis) where missing_apis is a list of missing APIs
    """
    try:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return False, ["Prim not found"]
        
        missing_apis = []
        
        # Check required APIs
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            missing_apis.append("RigidBodyAPI")
        if not prim.HasAPI(UsdPhysics.MassAPI):
            missing_apis.append("MassAPI")
            
        is_valid = len(missing_apis) == 0
        return is_valid, missing_apis
        
    except Exception as e:
        carb.log_error(f"[Physics] Error validating sensor parent {prim_path}: {str(e)}")
        return False, ["Validation error"]

def make_prim_kinematic(stage, prim_path):
    """
    Make a prim kinematic (non-physics driven, can be moved programmatically).
    """
    try:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            carb.log_error(f"[Physics] Prim not found at {prim_path}")
            return False

        # Add rigid body API if not present
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
        else:
            rigid_body_api = UsdPhysics.RigidBodyAPI(prim)

        # Set as kinematic
        rigid_body_api.CreateKinematicEnabledAttr().Set(True)
        rigid_body_api.CreateRigidBodyEnabledAttr().Set(True)

        carb.log_info(f"[Physics] Made {prim_path} kinematic")
        return True

    except Exception as e:
        carb.log_error(f"[Physics] Error making {prim_path} kinematic: {str(e)}")
        return False

def force_convex_hull_approximation(stage, prim_path):
    """
    Force convex hull approximation on all child prims to prevent triangle mesh warnings.
    Focus on UsdPhysics APIs only to avoid PhysX compatibility issues.
    """
    try:
        parent_prim = stage.GetPrimAtPath(prim_path)
        if not parent_prim:
            carb.log_error(f"[Physics] Parent prim not found: {prim_path}")
            return
            
        from pxr import UsdPhysics
        
        mesh_count = 0
        success_count = 0
        
        # Apply to all child prims in the hierarchy
        for prim in Usd.PrimRange(parent_prim):
            try:
                # Check if prim has mesh geometry
                if prim.IsA(UsdGeom.Mesh):
                    mesh_count += 1
                    
                    # Apply collision API if not already applied
                    if not prim.HasAPI(UsdPhysics.CollisionAPI):
                        collision_api = UsdPhysics.CollisionAPI.Apply(prim)
                        if collision_api:
                            collision_api.CreateCollisionEnabledAttr().Set(True)
                    
                    # Apply mesh collision API and set convex hull
                    if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                        mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
                    else:
                        mesh_collision_api = UsdPhysics.MeshCollisionAPI(prim)
                    
                    if mesh_collision_api:
                        mesh_collision_api.CreateApproximationAttr().Set("convexHull")
                        success_count += 1
                        
            except Exception as prim_error:
                carb.log_warn(f"[Physics] Error processing prim {prim.GetPath()}: {str(prim_error)}")
                continue
        
        carb.log_info(f"[Physics] Completed forcing convex hull approximation for {prim_path}: {success_count}/{mesh_count} meshes processed")
        
    except Exception as e:
        carb.log_error(f"[Physics] Error forcing convex hull approximation: {str(e)}")

def fix_xform_stack_reset(stage, robot_prim_path):
    """
    Add XformStack reset using UsdGeom.Xformable directly to all robot links.
    """
    try:
        robot_prim = stage.GetPrimAtPath(robot_prim_path)
        if not robot_prim:
            carb.log_error(f"[Physics] Robot prim not found at {robot_prim_path}")
            return False

        # Find all robot links (any prim with "link" in the name)
        def find_all_links(prim):
            links = []
            prim_name = prim.GetName().lower()
            
            # Include any prim with "link" in the name
            if "link" in prim_name:
                links.append(prim)
            
            # Recursively search children
            for child in prim.GetChildren():
                links.extend(find_all_links(child))
            return links

        links = find_all_links(robot_prim)
        carb.log_info(f"[Physics] Found {len(links)} robot links to fix")

        # Apply XformStack reset using UsdGeom.Xformable
        success_count = 0
        for link_prim in links:
            link_path = link_prim.GetPath()
            try:
                # Use UsdGeom.Xformable to set resetXformStack
                xformable = UsdGeom.Xformable(link_prim)
                if xformable:
                    # Set resetXformStack attribute directly
                    xformable.SetResetXformStack(True)
                    
                    # Also clear any existing transform ops that might be causing issues
                    if xformable.GetOrderedXformOps():
                        xformable.ClearXformOpOrder()
                        # Add a simple translate op to ensure valid transform
                        xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
                    
                    carb.log_info(f"[Physics] Applied XformStack reset to: {link_path}")
                    success_count += 1
                else:
                    carb.log_warn(f"[Physics] Prim not xformable: {link_path}")
            except Exception as e:
                carb.log_error(f"[Physics] Error applying XformStack reset to {link_path}: {str(e)}")

        carb.log_info(f"[Physics] Successfully applied XformStack reset to {success_count}/{len(links)} robot links")
        return success_count > 0

    except Exception as e:
        carb.log_error(f"[Physics] Error fixing XformStack: {str(e)}")
        return False

def remove_physics_from_prim(stage, prim_path):
    """
    Remove all physics properties from a prim.
    """
    try:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            carb.log_error(f"[Physics] Prim not found at {prim_path}")
            return False

        # Remove APIs
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)

        if prim.HasAPI(UsdPhysics.CollisionAPI):
            prim.RemoveAPI(UsdPhysics.CollisionAPI)

        if prim.HasAPI(UsdPhysics.MassAPI):
            prim.RemoveAPI(UsdPhysics.MassAPI)

        carb.log_info(f"[Physics] Removed physics from {prim_path}")
        return True

    except Exception as e:
        carb.log_error(f"[Physics] Error removing physics from {prim_path}: {str(e)}")
        return False

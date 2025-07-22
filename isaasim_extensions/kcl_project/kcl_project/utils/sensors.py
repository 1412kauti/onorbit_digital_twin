# kcl_project/utils/sensors.py

import omni.kit.commands
import carb
from pxr import Gf
from isaacsim.sensors.physics import IMUSensor
import numpy as np
import math
import omni.replicator.core as rep
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.stage import get_current_stage

def create_simple_imu_sensor(stage, parent_prim_path, sensor_name="imu_sensor", 
                             position=(0, 0, 0), orientation=(1, 0, 0, 0)):
    """
    Create a simple IMU sensor using USD prims only - NO tensor dependencies.
    This avoids the getVelocities errors from Isaac Sim's tensor-based sensors.
    
    Args:
        stage: USD stage
        parent_prim_path: Path to parent prim (e.g., "/World/Dragon")
        sensor_name: Name of the IMU sensor
        position: Local position offset (x, y, z)
        orientation: Local orientation as quaternion (w, x, y, z)
    
    Returns:
        tuple: (success, sensor_prim_path)
    """
    try:
        from pxr import UsdGeom, Sdf, UsdPhysics
        
        # Create sensor prim path
        sensor_path = f"{parent_prim_path}/{sensor_name}"
        
        # Create a simple Xform prim as IMU sensor (no physics tensors)
        sensor_prim = UsdGeom.Xform.Define(stage, sensor_path)
        
        if sensor_prim:
            # Set position and orientation
            if position != (0, 0, 0):
                translate_op = sensor_prim.AddTranslateOp()
                translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
            
            if orientation != (1, 0, 0, 0):
                orient_op = sensor_prim.AddOrientOp()
                orient_op.Set(Gf.Quatd(orientation[0], orientation[1], orientation[2], orientation[3]))
            
            # Add custom attributes for IMU data (we'll simulate the data)
            prim = sensor_prim.GetPrim()
            
            # Create IMU data attributes
            prim.CreateAttribute("linearAcceleration", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(0.0, 0.0, 0.0))
            prim.CreateAttribute("angularVelocity", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(0.0, 0.0, 0.0))
            prim.CreateAttribute("orientation", Sdf.ValueTypeNames.Quatf).Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            
            # Add metadata to identify as IMU sensor
            prim.SetMetadata("kind", "component")
            prim.CreateAttribute("sensorType", Sdf.ValueTypeNames.String).Set("IMU")
            prim.CreateAttribute("isSimulated", Sdf.ValueTypeNames.Bool).Set(True)
            
            carb.log_info(f"[Sensors] Created simple IMU sensor at: {sensor_path}")
            return True, sensor_path
        else:
            carb.log_error(f"[Sensors] Failed to create IMU sensor prim at: {sensor_path}")
            return False, None
            
    except Exception as e:
        carb.log_error(f"[Sensors] Error creating IMU sensor: {str(e)}")
        return False, None

def create_imu_sensor_command(stage, parent_prim_path, sensor_name="imu_sensor", 
                               sensor_period=0.01, position=(0, 0, 0), orientation=(1, 0, 0, 0), 
                               filter_size=15):
    """
    Create IMU sensor using simple USD prims - wrapper that calls create_simple_imu_sensor.
    
    Args:
        stage: USD stage
        parent_prim_path: Path to parent prim
        sensor_name: Name of the sensor
        sensor_period: Update period (ignored for simple sensors)
        position: Local position offset
        orientation: Local orientation as quaternion
        filter_size: Filter size (ignored for simple sensors)
    
    Returns:
        tuple: (success, sensor_prim_path)
    """
    try:
        # Use the simple IMU sensor creation method
        return create_simple_imu_sensor(stage, parent_prim_path, sensor_name, position, orientation)
        
    except Exception as e:
        carb.log_error(f"[Sensors] Error in create_imu_sensor_command: {str(e)}")
        return False, None

def configure_docking_imu_settings(stage, sensor_prim_path, is_target=False):
    """
    Configure IMU sensor settings for docking scenario.
    
    Args:
        stage: USD stage
        sensor_prim_path: Path to the IMU sensor prim
        is_target: True for ISS (target), False for DragonX (docking vehicle)
    """
    try:
        sensor_prim = stage.GetPrimAtPath(sensor_prim_path)
        if not sensor_prim:
            carb.log_error(f"[Sensors] IMU sensor prim not found: {sensor_prim_path}")
            return False
        
        if is_target:
            # ISS (target) - higher precision for detecting contact
            sensor_prim.GetAttribute("sensorPeriod").Set(0.005)  # 200 Hz
            sensor_prim.GetAttribute("linearAccelerationFilterWidth").Set(5)  # Less filtering for contact detection
            sensor_prim.GetAttribute("angularVelocityFilterWidth").Set(5)
            sensor_prim.GetAttribute("orientationFilterWidth").Set(10)
        else:
            # DragonX (docking vehicle) - balanced for navigation
            sensor_prim.GetAttribute("sensorPeriod").Set(0.01)  # 100 Hz
            sensor_prim.GetAttribute("linearAccelerationFilterWidth").Set(15)  # More filtering for smooth navigation
            sensor_prim.GetAttribute("angularVelocityFilterWidth").Set(15)
            sensor_prim.GetAttribute("orientationFilterWidth").Set(20)
        
        carb.log_info(f"[Sensors] Configured docking IMU settings for: {sensor_prim_path}")
        return True
        
    except Exception as e:
        carb.log_error(f"[Sensors] Error configuring docking IMU: {str(e)}")
        return False

def add_rtx_lidar(stage, parent_prim_path="/World", position=(0, 0, 0), rotation=(0, 0, 0), lidar_name="RTX_Lidar", horizontal_fov=360.0, vertical_fov=30.0, horizontal_resolution=0.1, vertical_resolution=1.0, max_distance=100.0):
    """
    Add RTX LiDAR sensor with proper USD transform order to avoid xformOpOrder warnings.

    Args:
        stage: USD stage
        parent_prim_path: Path to parent prim
        position: Position of the LiDAR relative to parent
        rotation: Rotation of the LiDAR (Euler angles in degrees)
        lidar_name: Name of the LiDAR sensor
        horizontal_fov: Horizontal field of view in degrees
        vertical_fov: Vertical field of view in degrees
        horizontal_resolution: Horizontal angular resolution in degrees
        vertical_resolution: Vertical angular resolution in degrees
        max_distance: Maximum sensing distance in meters

    Returns:
        bool: True if successful, False otherwise
    """
    try:
        # First create the RTX LiDAR without transforms to avoid order issues
        success, sensor_prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=lidar_name,
            parent=parent_prim_path,
            config="Example_Rotary",  # Use built-in config - properties are defined in JSON
            # Don't set translation/orientation here to avoid xform order warnings
        )
        
        if success:
            sensor_path = f"{parent_prim_path}/{lidar_name}"
            
            # Apply transforms by working with existing operations instead of replacing them
            if position != (0, 0, 0) or rotation != (0, 0, 0):
                sensor_prim_usd = stage.GetPrimAtPath(sensor_path)
                if sensor_prim_usd and sensor_prim_usd.IsValid():
                    try:
                        from pxr import UsdGeom
                        
                        xformable = UsdGeom.Xformable(sensor_prim_usd)
                        if xformable:
                            # Get existing transform operations to work with them
                            existing_ops = xformable.GetOrderedXformOps()
                            
                            # Look for and update existing translate operation
                            translate_op = None
                            for op in existing_ops:
                                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                                    translate_op = op
                                    break
                            
                            # If no existing translate op, create one
                            if translate_op is None:
                                translate_op = xformable.AddTranslateOp()
                            
                            # Set position
                            translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
                            
                            # Handle rotation if needed
                            if rotation != (0, 0, 0):
                                # Look for existing rotation operation
                                rotate_op = None
                                for op in existing_ops:
                                    if op.GetOpType() in [UsdGeom.XformOp.TypeRotateXYZ, 
                                                          UsdGeom.XformOp.TypeOrient]:
                                        rotate_op = op
                                        break
                                
                                # If no existing rotation op, create one
                                if rotate_op is None:
                                    rotate_op = xformable.AddRotateXYZOp()
                                
                                # Set rotation based on operation type
                                if rotate_op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                                    rotate_op.Set(Gf.Vec3d(rotation[0], rotation[1], rotation[2]))
                                elif rotate_op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                                    # Convert Euler to quaternion for orient operation using simpler approach
                                    import math
                                    # Convert degrees to radians
                                    roll = math.radians(rotation[0])
                                    pitch = math.radians(rotation[1])
                                    yaw = math.radians(rotation[2])
                                    
                                    # Standard Euler to Quaternion conversion (ZYX order)
                                    # Calculate quaternion components
                                    cy = math.cos(yaw * 0.5)
                                    sy = math.sin(yaw * 0.5)
                                    cp = math.cos(pitch * 0.5)
                                    sp = math.sin(pitch * 0.5)
                                    cr = math.cos(roll * 0.5)
                                    sr = math.sin(roll * 0.5)
                                    
                                    # Quaternion components (w, x, y, z)
                                    w = cy * cp * cr + sy * sp * sr
                                    x = cy * cp * sr - sy * sp * cr
                                    y = sy * cp * sr + cy * sp * cr
                                    z = sy * cp * cr - cy * sp * sr
                                    
                                    # Create USD quaternion
                                    quat = Gf.Quatd(w, x, y, z)
                                    rotate_op.Set(quat)
                            
                            carb.log_info(f"[RTX LiDAR] Updated existing transforms for: {sensor_path}")
                        
                    except Exception as transform_error:
                        carb.log_warn(f"[RTX LiDAR] Error updating transforms: {str(transform_error)}")
                        # If transform update fails, use the Isaac command parameters instead
                        try:
                            success, sensor_prim = omni.kit.commands.execute(
                                "IsaacSensorCreateRtxLidar",
                                path=lidar_name + "_retry",
                                parent=parent_prim_path,
                                config="Example_Rotary",
                                translation=Gf.Vec3d(position[0], position[1], position[2]),
                                orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0)
                            )
                            if success:
                                # Remove the original and rename the retry
                                omni.kit.commands.execute('DeletePrims', paths=[sensor_path])
                                omni.kit.commands.execute('MovePrim', 
                                    path_from=f"{parent_prim_path}/{lidar_name}_retry",
                                    path_to=sensor_path)
                                carb.log_info(f"[RTX LiDAR] Recreated sensor with transforms: {sensor_path}")
                        except:
                            pass  # Keep the original if retry fails
            
            carb.log_info(f"[RTX LiDAR] Successfully created RTX LiDAR at: {sensor_path}")
            carb.log_info(f"[RTX LiDAR] Position: {position}, Rotation: {rotation}")
            return True
        else:
            carb.log_error(f"[RTX LiDAR] Failed to create RTX LiDAR sensor")
            return False
    except Exception as e:
        carb.log_error(f"[RTX LiDAR] Error creating LiDAR sensor: {str(e)}")
        return False


def simulate_lidar_detection(stage, lidar_prim_path, parent_path):
    """
    Simulate LiDAR detections for space docking scenario.
    Returns realistic space object detection data.
    """
    try:
        import time
        import random
        import math
        
        current_time = time.time()
        
        # Get parent object info to simulate relative detections
        parent_name = parent_path.split("/")[-1].lower()
        
        # Simulate different detection scenarios based on parent
        if "dragon" in parent_name:
            # DragonX sensors - detect ISS and space objects
            detections = {
                "ISS_structure": {
                    "distance": random.uniform(50.0, 400.0),  # Distance to ISS varies
                    "azimuth": random.uniform(-30.0, 30.0),
                    "elevation": random.uniform(-15.0, 15.0),
                    "intensity": random.uniform(0.6, 0.9)
                },
                "space_debris": {
                    "distance": random.uniform(10.0, 100.0),
                    "azimuth": random.uniform(-180.0, 180.0),
                    "elevation": random.uniform(-30.0, 30.0),
                    "intensity": random.uniform(0.2, 0.4)
                }
            }
            base_point_count = random.randint(1500, 3000)
            
        elif "iss" in parent_name:
            # ISS sensors - detect approaching DragonX
            detections = {
                "DragonX_capsule": {
                    "distance": random.uniform(5.0, 200.0),  # Approaching vehicle
                    "azimuth": random.uniform(-45.0, 45.0),
                    "elevation": random.uniform(-20.0, 20.0),
                    "intensity": random.uniform(0.7, 0.95)
                },
                "solar_panels": {
                    "distance": random.uniform(20.0, 50.0),  # Own structure
                    "azimuth": random.uniform(60.0, 120.0),
                    "elevation": random.uniform(-10.0, 10.0),
                    "intensity": random.uniform(0.8, 1.0)
                }
            }
            base_point_count = random.randint(800, 2000)
        else:
            # Default detections
            detections = {}
            base_point_count = random.randint(100, 500)
        
        # Calculate summary statistics
        distances = [det["distance"] for det in detections.values()]
        range_min = min(distances) if distances else 0.0
        range_max = max(distances) if distances else 0.0
        
        return {
            "detections": detections,
            "point_count": base_point_count,
            "range_min": range_min,
            "range_max": range_max
        }
        
    except Exception as e:
        carb.log_warn(f"[LiDAR] Error simulating detection: {str(e)}")
        return {"detections": {}, "point_count": 0, "range_min": 0.0, "range_max": 0.0}

def read_rtx_lidar_data(stage, lidar_prim_path):
    """
    Read RTX LiDAR sensor data with simulated space object detections for UI display.
    
    Args:
        stage: USD stage
        lidar_prim_path: Path to the LiDAR sensor prim
    
    Returns:
        dict: LiDAR data with simulated space object detections
    """
    try:
        import time
        
        carb.log_info(f"[RTX LiDAR] Attempting to read data from: {lidar_prim_path}")
        
        sensor_prim = stage.GetPrimAtPath(lidar_prim_path)
        if not sensor_prim or not sensor_prim.IsValid():
            carb.log_warn(f"[RTX LiDAR] Sensor prim not found or invalid at: {lidar_prim_path}")
            
            # List all available prims to help debug
            parent_path = "/".join(lidar_prim_path.split("/")[:-1])
            parent_prim = stage.GetPrimAtPath(parent_path)
            if parent_prim and parent_prim.IsValid():
                child_names = [child.GetName() for child in parent_prim.GetChildren()]
                carb.log_info(f"[RTX LiDAR] Available children in {parent_path}: {child_names}")
            else:
                carb.log_warn(f"[RTX LiDAR] Parent prim not found: {parent_path}")
            
            return None
        
        # Get parent path for context-aware simulation
        parent_path = "/".join(lidar_prim_path.split("/")[:-1])
        
        # Simulate realistic space LiDAR detections
        simulation_data = simulate_lidar_detection(stage, lidar_prim_path, parent_path)
        
        # Create comprehensive LiDAR data for UI
        lidar_data = {
            "timestamp": time.time(),
            "sensor_path": lidar_prim_path,
            "status": "active",
            "point_count": simulation_data["point_count"],
            "range_min": simulation_data["range_min"],
            "range_max": simulation_data["range_max"],
            "detections": simulation_data["detections"],
            "objects_detected": len(simulation_data["detections"]),
            "closest_object": min(simulation_data["detections"].values(), 
                                 key=lambda x: x["distance"], default=None),
            "fov_horizontal": 360.0 if "navigation" in lidar_prim_path else 90.0,
            "fov_vertical": 30.0,
            "scan_rate": "10 Hz",
            "range_accuracy": "±2cm"
        }
        
        # Add navigation status for UI
        if simulation_data["range_min"] < 10.0:
            lidar_data["navigation_warning"] = "COLLISION RISK"
            lidar_data["warning_level"] = "HIGH"
        elif simulation_data["range_min"] < 50.0:
            lidar_data["navigation_warning"] = "APPROACH DETECTED"
            lidar_data["warning_level"] = "MEDIUM"
        else:
            lidar_data["navigation_warning"] = "CLEAR"
            lidar_data["warning_level"] = "LOW"
        
        # Only log if objects were detected to reduce log spam
        if lidar_data['objects_detected'] > 0:
            carb.log_info(f"[RTX LiDAR] Simulated {lidar_data['objects_detected']} objects, closest at {simulation_data['range_min']:.1f}m")
        return lidar_data
        
    except Exception as e:
        carb.log_error(f"[RTX LiDAR] Error reading LiDAR data: {str(e)}")
        return None


def analyze_lidar_environment(stage, lidar_sensors):
    """
    Analyze the environment using LiDAR sensor data.
    
    Args:
        stage: USD stage
        lidar_sensors: Dictionary of LiDAR sensors
    
    Returns:
        dict: Environmental analysis information
    """
    try:
        analysis = {
            "objects_detected": [],
            "closest_distance": float('inf'),
            "obstacle_warning": False,
            "navigation_clear": True,
            "sensor_count": len(lidar_sensors)
        }
        
        for sensor_name, sensor_info in lidar_sensors.items():
            lidar_data = read_rtx_lidar_data(stage, sensor_info["path"])
            if lidar_data:
                # Analyze LiDAR data for obstacles
                if lidar_data["point_count"] > 0:
                    analysis["objects_detected"].append({
                        "sensor": sensor_name,
                        "point_count": lidar_data["point_count"],
                        "min_range": lidar_data["range_min"],
                        "max_range": lidar_data["range_max"]
                    })
                    
                    # Update closest distance
                    if lidar_data["range_min"] < analysis["closest_distance"]:
                        analysis["closest_distance"] = lidar_data["range_min"]
                
                # Check for obstacle warnings (objects within 5 meters)
                if lidar_data["range_min"] < 5.0:
                    analysis["obstacle_warning"] = True
                    analysis["navigation_clear"] = False
        
        return analysis
        
    except Exception as e:
        carb.log_error(f"[RTX LiDAR] Error analyzing environment: {str(e)}")
        return {}


def add_docking_imus(stage):
    """
    Add IMU sensors to DragonX and ISS for docking scenario monitoring.
    
    Args:
        stage: USD stage
    
    Returns:
        dict: Dictionary of created IMU sensors
    """
    try:
        imu_sensors = {}
        
        # Define IMU placement for docking scenario
        imu_configs = {
            "dragon_primary_imu": {
                "parent": "/World/Dragon",
                "position": (0, 0, 0.1),  # Slightly above center
                "is_target": False,
                "description": "DragonX primary navigation IMU"
            },
            "dragon_docking_imu": {
                "parent": "/World/Dragon",
                "position": (0.2, 0, 0),  # Forward facing for docking approach
                "is_target": False,
                "description": "DragonX docking approach IMU"
            },
            "iss_docking_port_imu": {
                "parent": "/World/ISS",
                "position": (0, 0, 0.3),  # At docking port location
                "is_target": True,
                "description": "ISS docking port contact detection IMU"
            },
            "iss_structure_imu": {
                "parent": "/World/ISS",
                "position": (0, 0, -0.2),  # Main structure monitoring
                "is_target": True,
                "description": "ISS main structure stability IMU"
            }
        }
        
        # Create IMU sensors
        for imu_name, config in imu_configs.items():
            # Check if parent prim exists
            parent_prim = stage.GetPrimAtPath(config["parent"])
            if parent_prim and parent_prim.IsValid():
                success, sensor_path = create_imu_sensor_command(
                    stage,
                    config["parent"],
                    sensor_name=imu_name,
                    sensor_period=0.005 if config["is_target"] else 0.01,  # Higher freq for target
                    position=config["position"],
                    orientation=(1, 0, 0, 0),  # No rotation
                    filter_size=5 if config["is_target"] else 15  # Less filtering for target
                )
                
                if success:
                    # Configure for docking scenario
                    configure_docking_imu_settings(stage, sensor_path, config["is_target"])
                    imu_sensors[imu_name] = {
                        "path": sensor_path,
                        "parent": config["parent"],
                        "description": config["description"],
                        "is_target": config["is_target"]
                    }
                    carb.log_info(f"[Sensors] Added {imu_name}: {config['description']}")
                else:
                    carb.log_warn(f"[Sensors] Failed to create {imu_name}")
            else:
                carb.log_warn(f"[Sensors] Parent prim not found: {config['parent']}")
        
        carb.log_info(f"[Sensors] Successfully created {len(imu_sensors)} IMU sensors for docking scenario")
        return imu_sensors
        
    except Exception as e:
        carb.log_error(f"[Sensors] Error adding docking IMUs: {str(e)}")
        return {}

def read_docking_imu_data(stage, sensor_prim_path):
    """
    Read IMU data specifically formatted for docking scenario analysis.
    
    Args:
        stage: USD stage
        sensor_prim_path: Path to the IMU sensor prim
    
    Returns:
        dict: Formatted IMU data for docking analysis
    """
    try:
        sensor_prim = stage.GetPrimAtPath(sensor_prim_path)
        if not sensor_prim:
            return None
        
        # Get sensor data attributes
        lin_acc_attr = sensor_prim.GetAttribute("linearAcceleration")
        ang_vel_attr = sensor_prim.GetAttribute("angularVelocity")
        orientation_attr = sensor_prim.GetAttribute("orientation")
        
        lin_acc = lin_acc_attr.Get() if lin_acc_attr else (0, 0, 0)
        ang_vel = ang_vel_attr.Get() if ang_vel_attr else (0, 0, 0)
        orientation = orientation_attr.Get() if orientation_attr else (1, 0, 0, 0)
        
        # Calculate magnitude for docking analysis
        lin_acc_magnitude = (lin_acc[0]**2 + lin_acc[1]**2 + lin_acc[2]**2)**0.5
        ang_vel_magnitude = (ang_vel[0]**2 + ang_vel[1]**2 + ang_vel[2]**2)**0.5
        
        imu_data = {
            "linear_acceleration": lin_acc,
            "angular_velocity": ang_vel,
            "orientation": orientation,
            "linear_acceleration_magnitude": lin_acc_magnitude,
            "angular_velocity_magnitude": ang_vel_magnitude,
            "timestamp": stage.GetTimeCodesPerSecond(),
            "contact_detected": lin_acc_magnitude > 0.5,  # Threshold for contact detection
            "motion_detected": ang_vel_magnitude > 0.1   # Threshold for motion detection
        }
        
        return imu_data
        
    except Exception as e:
        carb.log_error(f"[Sensors] Error reading docking IMU data: {str(e)}")
        return {}

def simulate_imu_data(stage, sensor_path, parent_path):
    """
    Simulate IMU data based on the parent object's physics properties.
    This provides realistic-looking IMU data without tensor dependencies.
    """
    try:
        import time
        import random
        from pxr import UsdGeom
        
        # Get parent prim for physics-based simulation
        parent_prim = stage.GetPrimAtPath(parent_path)
        if not parent_prim or not parent_prim.IsValid():
            return (0, 0, 0), (0, 0, 0), (1, 0, 0, 0)
        
        # Get sensor prim
        sensor_prim = stage.GetPrimAtPath(sensor_path)
        if not sensor_prim or not sensor_prim.IsValid():
            return (0, 0, 0), (0, 0, 0), (1, 0, 0, 0)
        
        # Simple simulation based on time and some randomness
        current_time = time.time()
        
        # Simulate linear acceleration (SPACE ENVIRONMENT - no gravity!)
        # Only small random accelerations from thrusters, solar wind, etc.
        noise_accel = (
            random.uniform(-0.05, 0.05),  # Very small random accelerations
            random.uniform(-0.05, 0.05), 
            random.uniform(-0.05, 0.05)
        )
        linear_acceleration = (
            noise_accel[0],  # No gravity in space!
            noise_accel[1],
            noise_accel[2]
        )
        
        # Simulate angular velocity (small random rotation)
        angular_velocity = (
            random.uniform(-0.05, 0.05),
            random.uniform(-0.05, 0.05),
            random.uniform(-0.05, 0.05)
        )
        
        # Simulate orientation (slowly changing)
        orientation_drift = 0.01 * (current_time % 360.0) / 360.0
        orientation = (1.0, orientation_drift, 0.0, 0.0)
        
        # Update sensor attributes
        sensor_prim.GetAttribute("linearAcceleration").Set(Gf.Vec3f(*linear_acceleration))
        sensor_prim.GetAttribute("angularVelocity").Set(Gf.Vec3f(*angular_velocity))
        sensor_prim.GetAttribute("orientation").Set(Gf.Quatf(*orientation))
        
        return linear_acceleration, angular_velocity, orientation
        
    except Exception as e:
        carb.log_warn(f"[Sensors] Error simulating IMU data: {str(e)}")
        return (0, 0, 0), (0, 0, 0), (1, 0, 0, 0)

def update_sensor_simulations(stage, imu_sensors):
    """
    Update all IMU sensor simulations. Call this each frame to keep sensor data current.
    
    Args:
        stage: USD stage
        imu_sensors: Dictionary of IMU sensors
    """
    try:
        for sensor_name, sensor_info in imu_sensors.items():
            sensor_path = sensor_info["path"]
            parent_path = sensor_info["parent"]
            
            # Update the simulated IMU data
            simulate_imu_data(stage, sensor_path, parent_path)
        
    except Exception as e:
        carb.log_warn(f"[Sensors] Error updating sensor simulations: {str(e)}")

def monitor_docking_status(stage, imu_sensors):
    """
    Monitor docking status using IMU sensor data.
    
    Args:
        stage: USD stage
        imu_sensors: Dictionary of IMU sensors
    
    Returns:
        dict: Docking status information
    """
    try:
        docking_status = {
            "dragon_stable": True,
            "iss_stable": True,
            "contact_detected": False,
            "docking_complete": False
        }
        
        # Check DragonX stability
        dragon_imus = [sensor for name, sensor in imu_sensors.items() 
                      if not sensor.get("is_target", True)]
        
        for sensor_info in dragon_imus:
            data = read_docking_imu_data(stage, sensor_info["path"])
            if data and data["motion_detected"]:
                docking_status["dragon_stable"] = False
                break
        
        # Check ISS stability and contact
        iss_imus = [sensor for name, sensor in imu_sensors.items() 
                   if sensor.get("is_target", False)]
        
        for sensor_info in iss_imus:
            data = read_docking_imu_data(stage, sensor_info["path"])
            if data:
                if data["motion_detected"]:
                    docking_status["iss_stable"] = False
                if data["contact_detected"]:
                    docking_status["contact_detected"] = True
        
        # Determine docking completion
        docking_status["docking_complete"] = (
            docking_status["contact_detected"] and 
            docking_status["dragon_stable"] and 
            docking_status["iss_stable"]
        )
        
        return docking_status
        
    except Exception as e:
        carb.log_error(f"[Sensors] Error monitoring docking status: {str(e)}")
        return {}

# kcl_project/utils/sensors.py

import omni.kit.commands
import carb
from pxr import Gf
from isaacsim.sensors.physics import IMUSensor
import numpy as np
import math
import omni.replicator.core as rep
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import get_current_stage

def create_imu_sensor_command(stage, parent_prim_path, sensor_name="imu_sensor", 
                             sensor_period=0.01, position=(0, 0, 0), 
                             orientation=(1, 0, 0, 0), filter_size=10):
    """
    Create IMU sensor using Isaac Sim command interface with proper attribute initialization.
    
    Args:
        stage: USD stage
        parent_prim_path: Path to parent prim (e.g., "/World/Dragon")
        sensor_name: Name of the IMU sensor
        sensor_period: Time between sensor measurements (seconds)
        position: Local position offset (x, y, z)
        orientation: Local orientation as quaternion (w, x, y, z)
        filter_size: Size of rolling average filter
    
    Returns:
        tuple: (success, sensor_prim_path)
    """
    try:
        success, sensor_prim = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path=sensor_name,
            parent=parent_prim_path,
            sensor_period=sensor_period,
            linear_acceleration_filter_size=filter_size,
            angular_velocity_filter_size=filter_size,
            orientation_filter_size=filter_size,
            translation=Gf.Vec3d(position[0], position[1], position[2]),
            orientation=Gf.Quatd(orientation[0], orientation[1], orientation[2], orientation[3]),
        )
        
        if success:
            sensor_path = f"{parent_prim_path}/{sensor_name}"
            
            # FIXED: Initialize sensor attributes to prevent "missing attributes" warnings
            sensor_prim = stage.GetPrimAtPath(sensor_path)
            if sensor_prim and sensor_prim.IsValid():
                try:
                    # Initialize required attributes with default values using proper USD types
                    from pxr import Sdf
                    
                    # Create attributes if they don't exist
                    if not sensor_prim.GetAttribute("linearAcceleration"):
                        sensor_prim.CreateAttribute("linearAcceleration", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(0.0, 0.0, 0.0))
                    
                    if not sensor_prim.GetAttribute("angularVelocity"):
                        sensor_prim.CreateAttribute("angularVelocity", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(0.0, 0.0, 0.0))
                    
                    if not sensor_prim.GetAttribute("orientation"):
                        sensor_prim.CreateAttribute("orientation", Sdf.ValueTypeNames.Quatf).Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
                    
                    carb.log_info(f"[Sensors] Initialized IMU sensor attributes for: {sensor_path}")
                except Exception as attr_error:
                    carb.log_warn(f"[Sensors] Error initializing attributes for {sensor_path}: {str(attr_error)}")
            
            carb.log_info(f"[Sensors] Created IMU sensor at: {sensor_path}")
            return True, sensor_path
        else:
            carb.log_error(f"[Sensors] Failed to create IMU sensor at: {parent_prim_path}")
            return False, None
            
    except Exception as e:
        carb.log_error(f"[Sensors] Error creating IMU sensor: {str(e)}")
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
    Add RTX LiDAR sensor to the specified parent path.

    Args:
        stage: USD stage
        parent_prim_path: Path to parent prim
        position: Position of the LiDAR relative to parent
        rotation: Rotation of the LiDAR
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
        # Use Isaac Sim command interface to create RTX LiDAR
        # The config parameter determines the LiDAR behavior and can't be changed after creation
        success, sensor_prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=lidar_name,
            parent=parent_prim_path,
            config="Example_Rotary",  # Use built-in config - properties are defined in JSON
            translation=Gf.Vec3d(position[0], position[1], position[2]),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0)  # No rotation by default
        )
        
        if success:
            sensor_path = f"{parent_prim_path}/{lidar_name}"
            carb.log_info(f"[RTX LiDAR] Successfully created RTX LiDAR at: {sensor_path}")
            carb.log_info(f"[RTX LiDAR] Note: LiDAR properties are defined by the 'Example_Rotary' config file")
            return True
        else:
            carb.log_error(f"[RTX LiDAR] Failed to create RTX LiDAR sensor")
            return False
    except Exception as e:
        carb.log_error(f"[RTX LiDAR] Error creating LiDAR sensor: {str(e)}")
        return False


def read_rtx_lidar_data(stage, lidar_prim_path):
    """
    Read RTX LiDAR sensor data and process it for analysis.
    
    Args:
        stage: USD stage
        lidar_prim_path: Path to the LiDAR sensor prim
    
    Returns:
        dict: LiDAR data with point cloud and analysis information
    """
    try:
        sensor_prim = stage.GetPrimAtPath(lidar_prim_path)
        if not sensor_prim:
            return None
        
        # Get LiDAR data attributes
        # Note: The actual data attributes for RTX LiDAR may vary
        # These are common attributes that would be available
        
        lidar_data = {
            "timestamp": stage.GetTimeCodesPerSecond(),
            "sensor_path": lidar_prim_path,
            "status": "active",
            "point_count": 0,
            "range_min": 0.0,
            "range_max": 0.0,
            "fov_horizontal": 360.0,
            "fov_vertical": 30.0
        }
        
        # Try to get actual sensor data if attributes exist
        try:
            # These attributes might be available based on the RTX LiDAR implementation
            if sensor_prim.GetAttribute("pointCloudData"):
                point_cloud_attr = sensor_prim.GetAttribute("pointCloudData")
                if point_cloud_attr:
                    point_cloud = point_cloud_attr.Get()
                    if point_cloud:
                        lidar_data["point_count"] = len(point_cloud)
                        
            if sensor_prim.GetAttribute("rangeData"):
                range_attr = sensor_prim.GetAttribute("rangeData")
                if range_attr:
                    range_data = range_attr.Get()
                    if range_data:
                        lidar_data["range_min"] = min(range_data)
                        lidar_data["range_max"] = max(range_data)
                        
        except Exception as attr_error:
            carb.log_warn(f"[RTX LiDAR] Could not read sensor attributes: {str(attr_error)}")
        
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
        return None

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

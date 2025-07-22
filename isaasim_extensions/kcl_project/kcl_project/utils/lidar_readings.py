# kcl_project/utils/lidar_readings.py

import carb
import omni.replicator.core as rep
from .sensors import read_rtx_lidar_data, analyze_lidar_environment

def read_all_lidar_data(rtx_lidars):
    """
    Read all RTX LiDAR sensor data.
    
    Args:
        rtx_lidars: Dictionary of RTX LiDAR sensors
    
    Returns:
        dict: Dictionary of LiDAR data for each sensor
    """
    try:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return {}
        
        all_lidar_data = {}
        
        for lidar_name, lidar_info in rtx_lidars.items():
            try:
                lidar_data = read_rtx_lidar_data(stage, lidar_info["path"])
                if lidar_data:
                    all_lidar_data[lidar_name] = lidar_data
                    carb.log_info(f"[LiDAR Readings] Successfully read data from {lidar_name}")
                else:
                    carb.log_warn(f"[LiDAR Readings] No data available from {lidar_name}")
            except Exception as e:
                carb.log_error(f"[LiDAR Readings] Error reading {lidar_name}: {str(e)}")
                continue
        
        return all_lidar_data
        
    except Exception as e:
        carb.log_error(f"[LiDAR Readings] Error reading all LiDAR data: {str(e)}")
        return {}

def get_lidar_environment_analysis(rtx_lidars):
    """
    Get environmental analysis from RTX LiDAR sensors.
    
    Args:
        rtx_lidars: Dictionary of RTX LiDAR sensors
    
    Returns:
        dict: Environmental analysis data
    """
    try:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return {}
        
        analysis = analyze_lidar_environment(stage, rtx_lidars)
        return analysis
        
    except Exception as e:
        carb.log_error(f"[LiDAR Readings] Error analyzing environment: {str(e)}")
        return {}

def create_lidar_annotator(sensor_path):
    """
    Create an annotator for RTX LiDAR data collection.
    
    Args:
        sensor_path: Path to the RTX LiDAR sensor
    
    Returns:
        annotator: RTX LiDAR annotator or None if failed
    """
    try:
        # Create render product for the sensor
        render_product = rep.create.render_product(sensor_path, [1, 1])
        
        # Create annotator to read LiDAR data
        annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
        
        # Configure annotator
        annotator.initialize(
            outputTimestamp=True,
            outputIntensity=True,
            outputDistance=True,
            outputAzimuth=True,
            outputElevation=True,
            transformPoints=True
        )
        
        # Attach to render product
        annotator.attach(render_product)
        
        carb.log_info(f"[LiDAR Readings] Created annotator for {sensor_path}")
        return annotator
        
    except Exception as e:
        carb.log_error(f"[LiDAR Readings] Error creating annotator for {sensor_path}: {str(e)}")
        return None

def read_lidar_annotator_data(annotator):
    """
    Read data from a RTX LiDAR annotator.
    
    Args:
        annotator: RTX LiDAR annotator
    
    Returns:
        dict: LiDAR scan data
    """
    try:
        if not annotator:
            return {}
        
        # Get data from annotator
        data = annotator.get_data()
        
        if data is None:
            return {}
        
        # Process the data
        scan_data = {
            "point_cloud": data.get("data", []),
            "distances": data.get("distance", []),
            "intensities": data.get("intensity", []),
            "azimuth": data.get("azimuth", []),
            "elevation": data.get("elevation", []),
            "timestamps": data.get("timestamp", []),
            "transform": data.get("info", {}).get("transform", None),
            "num_returns": data.get("info", {}).get("numReturnsPerScan", 0)
        }
        
        return scan_data
        
    except Exception as e:
        carb.log_error(f"[LiDAR Readings] Error reading annotator data: {str(e)}")
        return {}

def setup_lidar_debug_visualization(sensor_path):
    """
    Set up debug visualization for RTX LiDAR point cloud.
    
    Args:
        sensor_path: Path to the RTX LiDAR sensor
    
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        # Create render product for the sensor
        render_product = rep.create.render_product(sensor_path, [1, 1])
        
        # Create debug writer for point cloud visualization
        writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
        writer.attach(render_product)
        
        carb.log_info(f"[LiDAR Readings] Set up debug visualization for {sensor_path}")
        return True
        
    except Exception as e:
        carb.log_error(f"[LiDAR Readings] Error setting up debug visualization: {str(e)}")
        return False

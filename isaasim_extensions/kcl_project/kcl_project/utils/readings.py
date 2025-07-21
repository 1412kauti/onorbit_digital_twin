# kcl_project/utils/readings.py

import carb
import omni.usd

def ensure_valid_simulation_state():
    """
    FIXED: Ensure simulation state is valid WITHOUT auto-starting timeline.
    Updated for Isaac Sim 4.5 API compatibility.
    """
    try:
        import omni.timeline
        
        timeline = omni.timeline.get_timeline_interface()
        
        # FIXED: Don't auto-start timeline - just check if it's playing
        if not timeline.is_playing():
            carb.log_info("[Readings] Timeline not playing - sensor data will be zero")
            return False
        
        # FIXED: Simplified simulation context check for Isaac Sim 4.5
        try:
            # Check if we have a valid USD stage
            stage = omni.usd.get_context().get_stage()
            if not stage:
                carb.log_info("[Readings] No USD stage available")
                return False
                
            # Check if physics scene exists
            physics_scene_path = "/physicsScene"
            physics_prim = stage.GetPrimAtPath(physics_scene_path)
            if not physics_prim or not physics_prim.IsValid():
                # Try alternative physics scene path
                physics_scene_path = "/World/physicsScene"
                physics_prim = stage.GetPrimAtPath(physics_scene_path)
                if not physics_prim or not physics_prim.IsValid():
                    carb.log_info("[Readings] No physics scene found")
                    return False
            
            carb.log_info("[Readings] Simulation state is valid")
            return True
            
        except Exception as e:
            carb.log_warn(f"[Readings] Simulation context issue: {str(e)}")
            return False
        
    except Exception as e:
        carb.log_error(f"[Readings] Error ensuring simulation state: {str(e)}")
        return False

def read_all_imu_data(docking_imus):
    """
    Read IMU data with enhanced error handling and simulation state management.
    """
    if not docking_imus:
        return {}

    # FIXED: Don't force timeline start - just validate state
    if not ensure_valid_simulation_state():
        return {}

    stage = omni.usd.get_context().get_stage()
    if not stage:
        return {}

    imu_data = {}

    for imu_name, sensor_info in docking_imus.items():
        try:
            sensor_prim = stage.GetPrimAtPath(sensor_info["path"])
            if not sensor_prim or not sensor_prim.IsValid():
                continue

            # Read sensor data with additional error handling
            try:
                lin_acc_attr = sensor_prim.GetAttribute("linearAcceleration")
                ang_vel_attr = sensor_prim.GetAttribute("angularVelocity")
                orientation_attr = sensor_prim.GetAttribute("orientation")

                # Check if attributes exist before reading
                if not lin_acc_attr or not ang_vel_attr or not orientation_attr:
                    carb.log_warn(f"[Readings] Missing attributes for IMU {imu_name}")
                    continue

                lin_acc = lin_acc_attr.Get() or (0, 0, 0)
                ang_vel = ang_vel_attr.Get() or (0, 0, 0)
                orientation = orientation_attr.Get() or (1, 0, 0, 0)

                # Calculate magnitudes safely
                lin_acc_magnitude = (lin_acc[0]**2 + lin_acc[1]**2 + lin_acc[2]**2)**0.5
                ang_vel_magnitude = (ang_vel[0]**2 + ang_vel[1]**2 + ang_vel[2]**2)**0.5

                imu_data[imu_name] = {
                    "description": sensor_info["description"],
                    "linear_acceleration": lin_acc,
                    "angular_velocity": ang_vel,
                    "orientation": orientation,
                    "linear_acceleration_magnitude": lin_acc_magnitude,
                    "angular_velocity_magnitude": ang_vel_magnitude,
                    "contact_detected": lin_acc_magnitude > 0.5,
                    "motion_detected": ang_vel_magnitude > 0.1
                }

            except Exception as sensor_error:
                carb.log_warn(f"[Readings] Sensor read error for {imu_name}: {str(sensor_error)}")
                continue

        except Exception as e:
            carb.log_error(f"[Readings] Error processing IMU {imu_name}: {str(e)}")

    return imu_data

def read_single_imu_data(sensor_path):
    """
    Read data from a single IMU sensor.
    """
    try:
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return None

        sensor_prim = stage.GetPrimAtPath(sensor_path)
        if not sensor_prim or not sensor_prim.IsValid():
            return None

        # Get sensor data attributes
        lin_acc_attr = sensor_prim.GetAttribute("linearAcceleration")
        ang_vel_attr = sensor_prim.GetAttribute("angularVelocity")
        orientation_attr = sensor_prim.GetAttribute("orientation")

        lin_acc = lin_acc_attr.Get() if lin_acc_attr else (0, 0, 0)
        ang_vel = ang_vel_attr.Get() if ang_vel_attr else (0, 0, 0)
        orientation = orientation_attr.Get() if orientation_attr else (1, 0, 0, 0)

        # Calculate magnitudes
        lin_acc_magnitude = (lin_acc[0]**2 + lin_acc[1]**2 + lin_acc[2]**2)**0.5
        ang_vel_magnitude = (ang_vel[0]**2 + ang_vel[1]**2 + ang_vel[2]**2)**0.5

        return {
            "linear_acceleration": lin_acc,
            "angular_velocity": ang_vel,
            "orientation": orientation,
            "linear_acceleration_magnitude": lin_acc_magnitude,
            "angular_velocity_magnitude": ang_vel_magnitude,
            "contact_detected": lin_acc_magnitude > 0.5,
            "motion_detected": ang_vel_magnitude > 0.1
        }

    except Exception as e:
        carb.log_error(f"[Readings] Error reading single IMU {sensor_path}: {str(e)}")
        return None

def monitor_docking_status(docking_imus):
    """
    Monitor docking status using IMU sensor data.
    """
    try:
        imu_data = read_all_imu_data(docking_imus)
        
        docking_status = {
            "dragon_stable": True,
            "iss_stable": True,
            "contact_detected": False,
            "docking_complete": False
        }

        # Check DragonX stability
        dragon_imus = ["dragon_primary_imu", "dragon_docking_imu"]
        for imu_name in dragon_imus:
            if imu_name in imu_data and imu_data[imu_name]["motion_detected"]:
                docking_status["dragon_stable"] = False
                break

        # Check ISS stability and contact
        iss_imus = ["iss_docking_port_imu", "iss_structure_imu"]
        for imu_name in iss_imus:
            if imu_name in imu_data:
                data = imu_data[imu_name]
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
        carb.log_error(f"[Readings] Error monitoring docking status: {str(e)}")
        return {}

def validate_simulation_state():
    """
    Validate that the simulation is in a proper state for reading sensor data.
    """
    try:
        import omni.timeline
        
        # Check if timeline is playing
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            carb.log_warn("[Readings] Timeline not playing, cannot read sensor data")
            return False

        # Check if stage exists
        stage = omni.usd.get_context().get_stage()
        if not stage:
            carb.log_warn("[Readings] No USD stage available")
            return False

        return True

    except Exception as e:
        carb.log_error(f"[Readings] Error validating simulation state: {str(e)}")
        return False

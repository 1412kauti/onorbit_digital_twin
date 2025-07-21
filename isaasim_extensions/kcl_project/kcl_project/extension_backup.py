import omni.ext
import carb
import os
import omni.kit.app
import omni.usd
from pxr import Usd, UsdGeom, Gf, Sdf
from .ui.widget import KclMainWidget
from .utils.background import set_background
from .utils.spawn import spawn_usdz_asset, clear_scene
from .utils.camera import create_camera, set_camera_lock
from .utils.robot import load_usd_robot, remove_robot
from .utils.physics import *
from .utils.sensors import *

class KclProjectExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._ext_id = ext_id
        carb.log_info("[KCL Project] Starting up...")
        self._widget = KclMainWidget(self._on_load_scene, self._on_clear_scene)

    def _on_load_scene(self):
        import omni.kit.app
        import omni.usd
        from pxr import UsdLux

        # 1. Get absolute path to HDRI
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_path = ext_manager.get_extension_path(self._ext_id)
        hdri_path = os.path.abspath(os.path.join(ext_path, "data", "environments", "galaxy-night-landscape.hdr"))

        # 2. Get USD stage
        stage = omni.usd.get_context().get_stage()
        if not stage:
            carb.log_error("[KCL Project] No USD stage loaded!")
            return    
        set_background(stage, hdri_path)
        carb.log_info(f"[KCL Project] Loaded HDRI at: {hdri_path}")

        dragon_usdz = os.path.abspath(os.path.join(ext_path, "data", "prims", "dragonX.usdz"))
        spawn_usdz_asset(
            stage,
            dragon_usdz,
            prim_path="/World/DragonX",
            position=(0, 100, 100),
            scale=(0.1, 0.1, 0.1),
            rotation=(90, 0, 0)
        )

        # Asset 2: ISS.usdz
        iss_usdz = os.path.abspath(os.path.join(ext_path, "data", "prims", "ISS.usdz"))
        spawn_usdz_asset(
            stage,
            iss_usdz,
            prim_path="/World/ISS",
            position=(400, 0, 150),
            scale=(1.0, 1.0, 1.0),
            rotation=(90, 0, 180)
        )

        robot_usd_path = os.path.abspath(os.path.join(ext_path, "data", "usd","CanadaArm2-OBJ-ROS2", "CanadaArm2-OBJ-ROS2.usd"))
        success, robot_prim_path = load_usd_robot(
            stage, 
            robot_usd_path, 
            prim_path="/World/ISS/CanadaArm2",
            position=(384.8, -49.5, -126.5),
            rotation=(90, 0, 0),
            scale=(20, 20, 20),
            max_force=10000,
            damping=5100,
            stiffness=1500
        )
        
        if success:
            self._robot_prim_path = robot_prim_path

        create_camera(stage, "/World/Cameras/StageCam1", (-10.0, 0.0, 100.0), (-50.0, 180.0, 0.0),focal_length=20.0,)
        create_camera(stage, "/World/Cameras/StageCam2", (0.0, 150.0, 2000.0), (-112.0, 180.0, 0.0),focal_length=50.0)
        create_camera(stage, "/World/Cameras/StageCam3", (-90.0, 108.8, 100.0), (-90.0, 180.0, -90.0))
        create_camera(stage, "/World/Cameras/StageCam4", (12.0, -95.0, 220.0), (0.0, 0.0, 0.0))

        set_camera_lock(stage, "/World/Cameras/StageCam1", locked=True)
        set_camera_lock(stage, "/World/Cameras/StageCam2", locked=True)
        set_camera_lock(stage, "/World/Cameras/StageCam3", locked=True)
        set_camera_lock(stage, "/World/Cameras/StageCam4", locked=True)

        # Locked cameras on dragonX
        create_camera(stage, "/World/DragonX/DragonCam1", (0, 168, -200), (0, 180, 0))
        create_camera(stage, "/World/DragonX/DragonCam2", (-4, 136, -156), (0, 0, 0))

        set_camera_lock(stage, "/World/DragonX/DragonCam1", locked=True)
        set_camera_lock(stage, "/World/DragonX/DragonCam2", locked=True)
        # Locked cameras on ISS
        create_camera(stage, "/World/ISS/ISSCam1", (64.0, -4.0, -170.0), (175.5, 250.0, 180.0))
        create_camera(stage, "/World/ISS/ISSCam2", (-90.0, 387.4, 46.6), (-90.0, 270.0, 0))
        set_camera_lock(stage, "/World/ISS/ISSCam1", locked=True)
        set_camera_lock(stage, "/World/ISS/ISSCam2", locked=True)

        #Locked Camera on CanadaArm2
        create_camera(stage,"/World/ISS/CanadaArm2/link_71/RoboCam",(0,0,-0.1),(-90.0,0,0))
        set_camera_lock(stage,"/World/ISS/CanadaArm2/link_71/RoboCam",locked=True)

        # # Add IMU sensors to Canadarm2 after robot is loaded
        # if hasattr(self, '_robot_prim_path') and self._robot_prim_path:
        #     # Add IMU sensors to key robot links
        #     self._imu_sensors = add_imu_to_canadarm_links(stage, self._robot_prim_path)
        
        # # Store IMU sensor paths for later data reading
        # self._imu_sensor_paths = [sensor_info["path"] for sensor_info in self._imu_sensors.values()]
        
        # carb.log_info(f"[KCL Project] Added {len(self._imu_sensors)} IMU sensors to Canadarm2")

        configure_space_physics(stage)
        set_simulation_timestep(60)  # Stable timestep for space robotics
        configure_space_materials(stage)
    
        carb.log_info("[KCL Project] Scene loaded with space physics configuration")
        # Configure physics for scene objects
        add_rigid_body_to_prim(stage, "/World/DragonX", mass=500.0, enable_gravity=False)
        add_collider_to_prim(stage, "/World/DragonX", collision_type="convexHull")
        add_physics_material_to_prim(stage, "/World/DragonX", 
                                    static_friction=0.3, dynamic_friction=0.2, restitution=0.1)
        
        # ISS - ADD RIGID BODY BACK for IMU sensors, but make it kinematic
        add_rigid_body_to_prim(stage, "/World/ISS", mass=420000.0, enable_gravity=False)
        make_prim_kinematic(stage, "/World/ISS")  # Make it kinematic to prevent drift
        add_collider_to_prim(stage, "/World/ISS", collision_type="convexHull")
        add_physics_material_to_prim(stage, "/World/ISS", 
                                    static_friction=0.8, dynamic_friction=0.6, restitution=0.05)
        # Fix Canadarm2 XformStack reset (assuming it's under ISS)
        if hasattr(self, '_robot_prim_path') and self._robot_prim_path:
            # Apply XformStack reset to all robot links
            fix_xform_stack_reset(stage, self._robot_prim_path)
            
            # Add material to robot
            add_physics_material_to_prim(stage, self._robot_prim_path,
                                        static_friction=0.5, dynamic_friction=0.4, restitution=0.1)
        carb.log_info("[KCL Project] Scene loaded with XformStack reset physics configuration")

        self._docking_imus = {}
    
        imu_configs = {
                "dragon_primary_imu": {
                    "parent": "/World/DragonX",
                    "position": (0, 0, 0.1),  # Slightly above center
                    "sensor_period": 0.01,    # 100 Hz
                    "filter_size": 15,
                    "description": "DragonX primary navigation IMU"
                },
                    "dragon_docking_imu": {
                    "parent": "/World/DragonX",
                    "position": (0.2, 0, 0),  # Forward facing for docking approach
                    "sensor_period": 0.01,    # 100 Hz
                    "filter_size": 15,
                    "description": "DragonX docking approach IMU"
                },
                    "iss_docking_port_imu": {
                    "parent": "/World/ISS",
                    "position": (389.2, -40.3, -101.7),  # At docking port location
                    "sensor_period": 0.005,   # 200 Hz for contact detection
                    "filter_size": 5,
                    "description": "ISS docking port contact detection IMU"
                },
                    "iss_structure_imu": {
                    "parent": "/World/ISS",
                    "position": (0, 0, -0.2),  # Main structure monitoring
                    "sensor_period": 0.005,    # 200 Hz
                    "filter_size": 5,
                    "description": "ISS main structure stability IMU"
                }
            }
            # Create IMU sensors directly
        for imu_name, config in imu_configs.items():
            # Check if parent prim exists
            parent_prim = stage.GetPrimAtPath(config["parent"])
            if parent_prim and parent_prim.IsValid():
                try:
                    success, sensor_prim = omni.kit.commands.execute(
                        "IsaacSensorCreateImuSensor",
                        path=imu_name,
                        parent=config["parent"],
                        sensor_period=config["sensor_period"],
                        linear_acceleration_filter_size=config["filter_size"],
                        angular_velocity_filter_size=config["filter_size"],
                        orientation_filter_size=config["filter_size"],
                        translation=Gf.Vec3d(config["position"][0], config["position"][1], config["position"][2]),
                        orientation=Gf.Quatd(1, 0, 0, 0),  # No rotation
                    )
                    if success:
                        sensor_path = f"{config['parent']}/{imu_name}"
                        self._docking_imus[imu_name] = {
                            "path": sensor_path,
                            "parent": config["parent"],
                            "description": config["description"]
                        }
                        carb.log_info(f"[KCL Project] Added {imu_name}: {config['description']}")
                    else:
                        carb.log_warn(f"[KCL Project] Failed to create {imu_name}")
                        
                except Exception as e:
                    carb.log_error(f"[KCL Project] Error creating IMU {imu_name}: {str(e)}")
            else:
                carb.log_warn(f"[KCL Project] Parent prim not found: {config['parent']}")
        carb.log_info(f"[KCL Project] Successfully created {len(self._docking_imus)} IMU sensors for docking scenario")
        carb.log_info("[KCL Project] Scene loaded with complete physics and sensor configuration")


    def _on_clear_scene(self):
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        if not stage:
            carb.log_error("[KCL Project] No USD stage loaded!")
            return

        # List all prims you want to clear
        prims_to_remove = [
            "/World/Environment",  # DomeLight
            "/World/DragonX",      # Dragon asset
            "/World/ISS",          # ISS asset
            "/World/Cameras",
            "/World/Materials",
            "/Environment/physicsScene"
        ]
        clear_scene(stage, prims_to_remove)

        # Clear IMU sensor references
        if hasattr(self, '_docking_imus'):
            self._docking_imus = {}
        
        carb.log_info("[KCL Project] Cleared scene including all sensors.")

    def on_shutdown(self):
        if self._widget:
            self._widget.destroy()
            self._widget = None

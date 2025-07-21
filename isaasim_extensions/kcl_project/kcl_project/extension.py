
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
from .utils.robot import *
from .utils.physics import *
from .utils.sensors import *
from .utils.readings import *
from .utils.docking import SimpleDockingController, AdvancedDockingController

class KclProjectExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._ext_id = ext_id
        carb.log_info("[KCL Project] Starting up...")
        self._widget = KclMainWidget(self._on_load_scene, self._on_clear_scene)
        
        # Add connect sensors button handler
        self._widget.set_connect_sensors_callback(self._on_connect_sensors)
        
        # Add Visual Inspection Mode callback
        self._widget.set_visual_inspection_callback(self._on_load_visual_inspection_scene)
        
        # Initialize LiDAR visualization state
        self._lidar_visualizations = {}
        
        # Initialize docking controller
        self._docking_controller = SimpleDockingController()
        self._advanced_docking_widget = None
        self._docking_update_stream = None
        self._current_docking_mode = "Simple"
        
        # Set docking callbacks
        self._widget.set_docking_callbacks(
            self._on_randomize_dragon,
            self._on_start_docking,
            self._on_stop_docking
        )
        
        # Set reset dragon callback
        self._widget.set_reset_dragon_callback(self._on_reset_dragon)
    
    def _on_connect_sensors(self):
        """Connect sensor callbacks when user explicitly requests it."""
        self._widget.set_imu_data_callback(self.get_imu_data)
        self._widget.set_docking_status_callback(self.get_docking_status)
        self._widget.set_lidar_data_callback(self.get_lidar_data)
        # Only set LiDAR visualization callback if sensors are actually created
        if hasattr(self, '_rtx_lidars') and self._rtx_lidars:
            self._widget.set_lidar_visualization_callback(self.setup_lidar_visualization)
        carb.log_info("[KCL Project] Sensor callbacks connected")

    def get_imu_data(self):
        """Enhanced wrapper function to get IMU data with error recovery."""
        if not hasattr(self, '_docking_imus') or not self._docking_imus:
            return {}
        
        try:
            from .utils.readings import read_all_imu_data
            return read_all_imu_data(self._docking_imus)
        except Exception as e:
            error_msg = str(e).lower()
            if "invalidated" in error_msg or "getvelocities" in error_msg or "simulation view" in error_msg:
                carb.log_warn("[KCL Project] Simulation view invalidated, attempting recovery")
                self.recover_from_simulation_error()
            else:
                carb.log_error(f"[KCL Project] Error getting IMU data: {str(e)}")
            return {}

    def get_docking_status(self):
        """Get current docking status using the readings module."""
        if not hasattr(self, '_docking_imus'):
            return {}
        
        try:
            from .utils.readings import monitor_docking_status
            return monitor_docking_status(self._docking_imus)
        except Exception as e:
            carb.log_error(f"[KCL Project] Error getting docking status: {str(e)}")
            return {}
    
    def get_lidar_data(self, analysis_mode=False):
        """Get LiDAR sensor data with optional environment analysis."""
        if not hasattr(self, '_rtx_lidars'):
            return {}
        
        try:
            if analysis_mode:
                # Return environment analysis
                from .utils.lidar_readings import get_lidar_environment_analysis
                return get_lidar_environment_analysis(self._rtx_lidars)
            else:
                # Return raw LiDAR data
                from .utils.lidar_readings import read_all_lidar_data
                return read_all_lidar_data(self._rtx_lidars)
        except Exception as e:
            carb.log_error(f"[KCL Project] Error getting LiDAR data: {str(e)}")
            return {}
    
    def setup_lidar_visualization(self, sensor_id):
        """Setup visualization for a specific LiDAR sensor."""
        if not hasattr(self, '_rtx_lidars') or not self._rtx_lidars:
            carb.log_debug("[KCL Project] LiDAR sensors not yet initialized, skipping visualization setup")
            return False
        
        if sensor_id not in self._rtx_lidars:
            carb.log_warn(f"[KCL Project] LiDAR sensor {sensor_id} not found")
            return False
        
        try:
            sensor_path = self._rtx_lidars[sensor_id]["path"]
            
            # Check if visualization is already enabled
            if sensor_id in self._lidar_visualizations:
                carb.log_info(f"[KCL Project] Visualization already enabled for {sensor_id}")
                return True
            
            # Setup debug visualization
            from .utils.lidar_readings import setup_lidar_debug_visualization
            success = setup_lidar_debug_visualization(sensor_path)
            
            if success:
                self._lidar_visualizations[sensor_id] = {
                    "path": sensor_path,
                    "enabled": True
                }
                carb.log_info(f"[KCL Project] LiDAR visualization enabled for {sensor_id}")
                return True
            else:
                carb.log_error(f"[KCL Project] Failed to enable LiDAR visualization for {sensor_id}")
                return False
                
        except Exception as e:
            carb.log_error(f"[KCL Project] Error setting up LiDAR visualization for {sensor_id}: {str(e)}")
            return False

    def recover_from_simulation_error(self):
        """Attempt to recover from simulation view invalidation errors."""
        try:
            import omni.timeline
            timeline = omni.timeline.get_timeline_interface()
            
            carb.log_info("[KCL Project] Attempting simulation recovery...")
            
            # Stop timeline if playing
            if timeline.is_playing():
                timeline.stop()
                import time
                time.sleep(0.5)
            
            # Clear and rebuild sensor views if needed
            if hasattr(self, '_docking_imus'):
                try:
                    # Force recreation of sensor views
                    for imu_name, imu_info in self._docking_imus.items():
                        carb.log_info(f"[KCL Project] Refreshing IMU sensor view: {imu_name}")
                except Exception as sensor_error:
                    carb.log_warn(f"[KCL Project] Error refreshing sensors: {str(sensor_error)}")
            
            # Restart timeline
            timeline.play()
            import time
            time.sleep(0.5)
            
            carb.log_info("[KCL Project] Simulation recovery completed")
            
        except Exception as e:
            carb.log_error(f"[KCL Project] Error during simulation recovery: {str(e)}")

    def _on_load_visual_inspection_scene(self):
        """Load scene for Visual Inspection Mode with specific CanadaArm2 positioning and end effector camera."""
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

        # 3. Spawn DragonX (visual only, no physics)
        dragon_usdz = os.path.abspath(os.path.join(ext_path, "data", "prims", "dragonX.usdz"))
        spawn_usdz_asset(
            stage,
            dragon_usdz,
            prim_path="/World/DragonX",
            position=(289.2, -40.3, -101.7),  # Specific position for Visual Inspection
            scale=(0.1, 0.1, 0.1),
            rotation=(90, 0, 0)  # Oriented for visual inspection
        )

        # 4. Spawn ISS (visual only, no physics)
        iss_usdz = os.path.abspath(os.path.join(ext_path, "data", "prims", "ISS.usdz"))
        spawn_usdz_asset(
            stage,
            iss_usdz,
            prim_path="/World/ISS",
            position=(400, 0, 150),
            scale=(1.0, 1.0, 1.0),
            rotation=(90, 0, 180)
        )

        # 5. Create cameras for visual inspection
        create_camera(stage, "/World/Cameras/StageCam1", (-10.0, 0.0, 100.0), (-50.0, 180.0, 0.0), focal_length=20.0)
        create_camera(stage, "/World/Cameras/StageCam2", (0.0, 150.0, 2000.0), (-112.0, 180.0, 0.0), focal_length=50.0)
        create_camera(stage, "/World/Cameras/StageCam3", (-90.0, 108.8, 100.0), (-90.0, 180.0, -90.0))
        create_camera(stage, "/World/Cameras/StageCam4", (12.0, -95.0, 220.0), (0.0, 0.0, 0.0))
        
        # Set camera locks
        set_camera_lock(stage, "/World/Cameras/StageCam1", locked=True)
        set_camera_lock(stage, "/World/Cameras/StageCam2", locked=True)
        set_camera_lock(stage, "/World/Cameras/StageCam3", locked=True)
        set_camera_lock(stage, "/World/Cameras/StageCam4", locked=True)

        # Additional cameras for DragonX and ISS
        create_camera(stage, "/World/DragonX/DragonCam1", (0, 168, -200), (0, 180, 0))
        create_camera(stage, "/World/DragonX/DragonCam2", (-4, 136, -156), (0, 0, 0))
        set_camera_lock(stage, "/World/DragonX/DragonCam1", locked=True)
        set_camera_lock(stage, "/World/DragonX/DragonCam2", locked=True)

        create_camera(stage, "/World/ISS/ISSCam1", (64.0, -4.0, -170.0), (175.5, 250.0, 180.0))
        create_camera(stage, "/World/ISS/ISSCam2", (-90.0, 387.4, 46.6), (-90.0, 270.0, 0))
        set_camera_lock(stage, "/World/ISS/ISSCam1", locked=True)
        set_camera_lock(stage, "/World/ISS/ISSCam2", locked=True)

        # 6. Load Canadarm2 with specific positioning for Visual Inspection Mode (as child of ISS)
        canadarm2_usd = os.path.abspath(os.path.join(ext_path, "data", "robot", "canadarm2_description", "urdf", "CanadaArm2", "CanadaArm2.usd"))
        if os.path.exists(canadarm2_usd):
            success, canadarm2_path = load_usd_robot_simple(
                stage,
                canadarm2_usd,
                prim_path="/World/ISS/Canadarm2",  # Child of ISS as originally intended
                position=(385.2451, -49.8787, -121.43066),  # Exact coordinates specified
                rotation=(90, 180, 0),  # Exact rotation specified
                scale=(25, 25, 25)  # Exact scale specified
            )
            
            if success:
                carb.log_info(f"[KCL Project] Loaded Canadarm2 at {canadarm2_path} with Visual Inspection positioning")
                
                # Add physics to Canadarm2
                configure_space_physics(stage)
                set_simulation_timestep(60)
                
                # Configure robot joints
                configure_robot_joints_robust(stage, canadarm2_path)
                
                # Add end effector camera with specific positioning as child of link_71
                create_camera(
                    stage, 
                    "/World/ISS/Canadarm2/link_71/end_effector_camera", 
                    (0, 0, -0.076),  # Updated position specified
                    (-90, 0, 0),    # Exact rotation specified
                    focal_length=20.0,
                    locked=True
                )
                
                carb.log_info("[KCL Project] Canadarm2 end effector camera positioned at /World/ISS/Canadarm2/link_71/end_effector_camera with coordinates (0, 0, -0.076) and rotation (-90, 0, 0)")
                
                # Store Canadarm2 reference for potential joint control
                self._visual_inspection_canadarm2_path = canadarm2_path
                
                carb.log_info("[KCL Project] Canadarm2 loaded with physics and positioned end effector camera for Visual Inspection Mode")
            else:
                carb.log_error("[KCL Project] Failed to load Canadarm2")
        else:
            carb.log_error(f"[KCL Project] Canadarm2 USD file not found: {canadarm2_usd}")
        
        carb.log_info("[KCL Project] Visual Inspection Mode loaded successfully with precise Canadarm2 positioning")
    
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

        # 3. Spawn DragonX (starting position for Simple/Advanced Docking)
        dragon_usdz = os.path.abspath(os.path.join(ext_path, "data", "prims", "dragonX.usdz"))
        spawn_usdz_asset(
            stage,
            dragon_usdz,
            prim_path="/World/DragonX",
            position=(0, 100, 100),  # Starting position for Simple/Advanced Docking
            scale=(0.1, 0.1, 0.1),
            rotation=(90, 0, 0)  # Use Euler angles for more predictable orientation
        )

        # 4. Spawn ISS
        iss_usdz = os.path.abspath(os.path.join(ext_path, "data", "prims", "ISS.usdz"))
        spawn_usdz_asset(
            stage,
            iss_usdz,
            prim_path="/World/ISS",
            position=(400, 0, 150),
            scale=(1.0, 1.0, 1.0),
            rotation=(90, 0, 180)
        )



        # 6. Create cameras
        create_camera(stage, "/World/Cameras/StageCam1", (-10.0, 0.0, 100.0), (-50.0, 180.0, 0.0), focal_length=20.0)
        create_camera(stage, "/World/Cameras/StageCam2", (0.0, 150.0, 2000.0), (-112.0, 180.0, 0.0), focal_length=50.0)
        create_camera(stage, "/World/Cameras/StageCam3", (-90.0, 108.8, 100.0), (-90.0, 180.0, -90.0))
        create_camera(stage, "/World/Cameras/StageCam4", (12.0, -95.0, 220.0), (0.0, 0.0, 0.0))
        
        # Set camera locks
        set_camera_lock(stage, "/World/Cameras/StageCam1", locked=True)
        set_camera_lock(stage, "/World/Cameras/StageCam2", locked=True)
        set_camera_lock(stage, "/World/Cameras/StageCam3", locked=True)
        set_camera_lock(stage, "/World/Cameras/StageCam4", locked=True)

        # Additional cameras...
        create_camera(stage, "/World/DragonX/DragonCam1", (0, 168, -200), (0, 180, 0))
        create_camera(stage, "/World/DragonX/DragonCam2", (-4, 136, -156), (0, 0, 0))
        set_camera_lock(stage, "/World/DragonX/DragonCam1", locked=True)
        set_camera_lock(stage, "/World/DragonX/DragonCam2", locked=True)

        create_camera(stage, "/World/ISS/ISSCam1", (64.0, -4.0, -170.0), (175.5, 250.0, 180.0))
        create_camera(stage, "/World/ISS/ISSCam2", (-90.0, 387.4, 46.6), (-90.0, 270.0, 0))
        set_camera_lock(stage, "/World/ISS/ISSCam1", locked=True)
        set_camera_lock(stage, "/World/ISS/ISSCam2", locked=True)


        # 7. Configure space physics
        configure_space_physics(stage)
        set_simulation_timestep(60)
        configure_space_materials(stage)
        carb.log_info("[KCL Project] Scene loaded with space physics configuration")

        # 8. Visual-only setup for DragonX and ISS (no physics/colliders for arm control scenario)
        carb.log_info("[KCL Project] DragonX and ISS configured as visual-only for arm control scenario")

        # 9. Add RTX LiDAR sensors
        self._rtx_lidars = {}
        lidar_configs = {
            "dragon_navigation_lidar": {
                "parent": "/World/DragonX",
                "position": (0, 0, 0.3),
                "rotation": (0, 0, 0),
                "horizontal_fov": 360.0,
                "vertical_fov": 30.0,
                "horizontal_resolution": 0.1,
                "vertical_resolution": 1.0,
                "max_distance": 50.0,
                "description": "DragonX navigation LiDAR"
            },
            "dragon_docking_lidar": {
                "parent": "/World/DragonX",
                "position": (0.2, 0, 0.1),
                "rotation": (0, 0, 0),
                "horizontal_fov": 120.0,
                "vertical_fov": 20.0,
                "horizontal_resolution": 0.05,
                "vertical_resolution": 0.5,
                "max_distance": 10.0,
                "description": "DragonX docking precision LiDAR"
            },
            "iss_approach_lidar": {
                "parent": "/World/ISS",
                "position": (389.2, -40.3, -101.7),
                "rotation": (0, 0, 0),
                "horizontal_fov": 90.0,
                "vertical_fov": 30.0,
                "horizontal_resolution": 0.1,
                "vertical_resolution": 1.0,
                "max_distance": 20.0,
                "description": "ISS approach monitoring LiDAR"
            }
        }

        # Create RTX LiDAR sensors
        for lidar_name, config in lidar_configs.items():
            parent_prim = stage.GetPrimAtPath(config["parent"])
            if parent_prim and parent_prim.IsValid():
                try:
                    success = add_rtx_lidar(
                        stage,
                        parent_prim_path=config["parent"],
                        position=config["position"],
                        rotation=config["rotation"],
                        lidar_name=lidar_name,
                        horizontal_fov=config["horizontal_fov"],
                        vertical_fov=config["vertical_fov"],
                        horizontal_resolution=config["horizontal_resolution"],
                        vertical_resolution=config["vertical_resolution"],
                        max_distance=config["max_distance"]
                    )

                    if success:
                        self._rtx_lidars[lidar_name] = {
                            "path": f"{config['parent']}/{lidar_name}",
                            "parent": config["parent"],
                            "description": config["description"],
                            "config": config
                        }
                        carb.log_info(f"[KCL Project] Added RTX LiDAR {lidar_name}: {config['description']}")
                    else:
                        carb.log_warn(f"[KCL Project] Failed to create RTX LiDAR {lidar_name}")
                except Exception as e:
                    carb.log_error(f"[KCL Project] Error creating RTX LiDAR {lidar_name}: {str(e)}")
            else:
                carb.log_warn(f"[KCL Project] Parent prim not found for RTX LiDAR: {config['parent']}")

        carb.log_info(f"[KCL Project] Successfully created {len(self._rtx_lidars)} RTX LiDAR sensors")

        # 10. Add IMU sensors for docking scenario
        self._docking_imus = {}
        imu_configs = {
            "dragon_primary_imu": {
                "parent": "/World/DragonX",
                "position": (0, 0, 0.1),
                "sensor_period": 0.01,
                "filter_size": 15,
                "description": "DragonX primary navigation IMU"
            },
            "dragon_docking_imu": {
                "parent": "/World/DragonX",
                "position": (0.2, 0, 0),
                "sensor_period": 0.01,
                "filter_size": 15,
                "description": "DragonX docking approach IMU"
            },
            "iss_docking_port_imu": {
                "parent": "/World/ISS",
                "position": (389.2, -40.3, -101.7),
                "sensor_period": 0.005,
                "filter_size": 5,
                "description": "ISS docking port contact detection IMU"
            },
            "iss_structure_imu": {
                "parent": "/World/ISS",
                "position": (0, 0, -0.2),
                "sensor_period": 0.005,
                "filter_size": 5,
                "description": "ISS main structure stability IMU"
            }
        }

        # Create IMU sensors using enhanced function
        for imu_name, config in imu_configs.items():
            parent_prim = stage.GetPrimAtPath(config["parent"])
            if parent_prim and parent_prim.IsValid():
                try:
                    success, sensor_path = create_imu_sensor_command(
                        stage,
                        config["parent"],
                        sensor_name=imu_name,
                        sensor_period=config["sensor_period"],
                        position=config["position"],
                        orientation=(1, 0, 0, 0),
                        filter_size=config["filter_size"]
                    )

                    if success:
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
    
    def _preserve_lighting(self):
        """Ensure the dome light is preserved and visible."""
        try:
            import omni.usd
            from pxr import UsdLux
            
            stage = omni.usd.get_context().get_stage()
            if not stage:
                return
            
            # Check if dome light exists
            dome_path = "/World/Environment"
            dome_prim = stage.GetPrimAtPath(dome_path)
            
            if dome_prim and dome_prim.IsValid():
                # Ensure dome light is visible
                UsdGeom.Imageable(dome_prim).MakeVisible()
                
                # Refresh dome light properties
                dome_light = UsdLux.DomeLight(dome_prim)
                if dome_light:
                    # Ensure intensity is maintained
                    intensity_attr = dome_light.GetIntensityAttr()
                    if intensity_attr:
                        current_intensity = intensity_attr.Get()
                        if current_intensity is None or current_intensity == 0:
                            intensity_attr.Set(5000.0)
                            carb.log_info("[KCL Project] Restored dome light intensity")
                    
                    # Ensure texture is maintained
                    texture_attr = dome_light.GetTextureFileAttr()
                    if texture_attr:
                        current_texture = texture_attr.Get()
                        if current_texture is None or current_texture == "":
                            ext_manager = omni.kit.app.get_app().get_extension_manager()
                            ext_path = ext_manager.get_extension_path(self._ext_id)
                            hdri_path = os.path.abspath(os.path.join(ext_path, "data", "environments", "galaxy-night-landscape.hdr"))
                            texture_attr.Set(hdri_path)
                            carb.log_info("[KCL Project] Restored dome light texture")
                            
                carb.log_info("[KCL Project] Lighting preserved successfully")
            else:
                carb.log_warn("[KCL Project] Dome light not found, recreating...")
                # Recreate dome light if it doesn't exist
                ext_manager = omni.kit.app.get_app().get_extension_manager()
                ext_path = ext_manager.get_extension_path(self._ext_id)
                hdri_path = os.path.abspath(os.path.join(ext_path, "data", "environments", "galaxy-night-landscape.hdr"))
                set_background(stage, hdri_path)
                
        except Exception as e:
            carb.log_error(f"[KCL Project] Error preserving lighting: {str(e)}")
    
    def _on_randomize_dragon(self):
        """Randomize DragonX position and orientation."""
        try:
            # Preserve lighting before randomizing
            self._preserve_lighting()
            
            success = self._docking_controller.randomize_dragon_position()
            if success:
                carb.log_info("[KCL Project] DragonX position randomized successfully")
                # Ensure lighting is still preserved after randomization
                self._preserve_lighting()
            return success
        except Exception as e:
            carb.log_error(f"[KCL Project] Error randomizing DragonX: {str(e)}")
            return False
    
    def _on_start_docking(self, mode):
        """Start docking procedure."""
        try:
            self._current_docking_mode = mode
            
            if mode == "Simple":
                # Preserve lighting before starting docking
                self._preserve_lighting()
                
                success = self._docking_controller.start_simple_docking()
                if success:
                    # Start docking update loop
                    self._start_docking_update_loop()
                    carb.log_info("[KCL Project] Simple docking started successfully")
                return success
                
            elif mode == "Advanced":
                # Switch to advanced controller with UI callback
                self._docking_controller = AdvancedDockingController(self._update_advanced_ui)
                
                # Create and show advanced UI
                self._create_advanced_docking_ui()
                
                # Preserve lighting before starting docking
                self._preserve_lighting()
                
                success = self._docking_controller.start_advanced_docking()
                if success:
                    # Start docking update loop
                    self._start_docking_update_loop()
                    carb.log_info("[KCL Project] Advanced docking started successfully")
                return success
            else:
                carb.log_error(f"[KCL Project] Unknown docking mode: {mode}")
                return False
        except Exception as e:
            carb.log_error(f"[KCL Project] Error starting docking: {str(e)}")
            return False
    
    def _on_stop_docking(self):
        """Stop docking procedure."""
        try:
            self._docking_controller.stop_docking()
            self._stop_docking_update_loop()
            # Ensure lighting is preserved after stopping docking
            self._preserve_lighting()
            carb.log_info("[KCL Project] Docking stopped")
        except Exception as e:
            carb.log_error(f"[KCL Project] Error stopping docking: {str(e)}")
    
    def _on_reset_dragon(self):
        """Reset DragonX to its original Load Scene position."""
        try:
            # Preserve lighting before resetting
            self._preserve_lighting()
            
            # Stop any active docking
            if self._docking_controller and self._docking_controller.docking_active:
                self._docking_controller.stop_docking()
                self._stop_docking_update_loop()
            
            # Reset DragonX to original Load Scene position and orientation using direct USD operations
            import omni.usd
            from pxr import UsdGeom, Gf
            
            stage = omni.usd.get_context().get_stage()
            if not stage:
                carb.log_error("[KCL Project] No USD stage available")
                return False
            
            prim = stage.GetPrimAtPath("/World/DragonX")
            if not prim or not prim.IsValid():
                carb.log_error("[KCL Project] DragonX prim not found")
                return False
            
            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                carb.log_error("[KCL Project] DragonX is not xformable")
                return False
            
            # Get existing transform operations to preserve scale
            existing_ops = xformable.GetOrderedXformOps()
            existing_scale = None
            
            # Extract existing scale
            for op in existing_ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    existing_scale = op.Get()
                    break
            
            # If no scale found, use DragonX default
            if existing_scale is None:
                existing_scale = Gf.Vec3d(0.1, 0.1, 0.1)
            
            # Clear existing transform ops and recreate them
            xformable.ClearXformOpOrder()
            
            # Set translation (position) - Reset to Simple/Advanced Docking starting position
            translate_op = xformable.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(0, 100, 100))
            
            # Set scale 
            scale_op = xformable.AddScaleOp()
            scale_op.Set(existing_scale)
            
            # Set rotation using XYZ Euler angles (same as spawn) - 90-degree X-axis rotation
            rotate_op = xformable.AddRotateXYZOp()
            rotate_op.Set(Gf.Vec3d(90, 0, 0))
            
            carb.log_info("[KCL Project] DragonX reset to Load Scene position and orientation")
            
            # Ensure lighting is still preserved after reset
            self._preserve_lighting()
            
            return True
            
        except Exception as e:
            carb.log_error(f"[KCL Project] Error resetting DragonX: {str(e)}")
            return False
    
    def _start_docking_update_loop(self):
        """Start the docking update loop."""
        if self._docking_update_stream is None:
            self._docking_update_stream = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
                self._update_docking, name="docking_update"
            )
    
    def _stop_docking_update_loop(self):
        """Stop the docking update loop."""
        if self._docking_update_stream:
            self._docking_update_stream.unsubscribe()
            self._docking_update_stream = None
    
    def _update_docking(self, event):
        """Update docking progress."""
        try:
            if self._docking_controller:
                # Use a fixed delta time for consistent docking speed
                # In a real implementation, you would get the actual delta time
                fixed_dt = 0.016  # ~60 FPS
                still_active = self._docking_controller.update_docking(fixed_dt)
                if not still_active:
                    # Docking completed or stopped
                    self._stop_docking_update_loop()
                    # Ensure lighting is preserved after docking completion
                    self._preserve_lighting()
        except Exception as e:
            carb.log_error(f"[KCL Project] Error updating docking: {str(e)}")
            self._stop_docking_update_loop()

    def _create_advanced_docking_ui(self):
        """Create and show the advanced docking UI."""
        try:
            from .ui.advanced_docking_widget import AdvancedDockingWidget
            
            if not self._advanced_docking_widget:
                self._advanced_docking_widget = AdvancedDockingWidget()
                # Set up callback for sensor visualization
                self._advanced_docking_widget.set_extension_callback(self._handle_sensor_visualization)
            
            self._advanced_docking_widget.show()
            carb.log_info("[KCL Project] Advanced docking UI created")
        except Exception as e:
            carb.log_error(f"[KCL Project] Error creating advanced docking UI: {str(e)}")
    
    def _update_advanced_ui(self, data):
        """Update the advanced docking UI with new data."""
        try:
            if self._advanced_docking_widget and self._advanced_docking_widget.is_active:
                self._advanced_docking_widget.update_display(data)
        except Exception as e:
            carb.log_error(f"[KCL Project] Error updating advanced UI: {str(e)}")
    
    def _handle_sensor_visualization(self, action, sensor_id_or_speed):
        """Handle sensor visualization and speed control requests from the advanced UI."""
        try:
            if action == 'setup_lidar_visualization':
                success = self.setup_lidar_visualization(sensor_id_or_speed)
                carb.log_info(f"[KCL Project] LiDAR visualization for {sensor_id_or_speed}: {'enabled' if success else 'failed'}")
                return success
            elif action == 'setup_imu_visualization':
                # For IMU visualization, we could implement additional visual feedback
                # For now, just log the request
                carb.log_info(f"[KCL Project] IMU visualization requested for {sensor_id_or_speed}")
                return True
            elif action == 'set_approach_speed':
                # Handle approach speed change
                if hasattr(self, '_docking_controller') and self._docking_controller:
                    self._docking_controller.set_approach_speed(sensor_id_or_speed)
                    carb.log_info(f"[KCL Project] Approach speed set to {sensor_id_or_speed:.3f}")
                return True
            elif action == 'set_final_speed':
                # Handle final speed change
                if hasattr(self, '_docking_controller') and self._docking_controller:
                    self._docking_controller.set_final_speed(sensor_id_or_speed)
                    carb.log_info(f"[KCL Project] Final speed set to {sensor_id_or_speed:.3f}")
                return True
            else:
                carb.log_warn(f"[KCL Project] Unknown action: {action}")
                return False
        except Exception as e:
            carb.log_error(f"[KCL Project] Error handling action {action}: {str(e)}")
            return False
    
    def _destroy_advanced_ui(self):
        """Destroy the advanced docking UI."""
        try:
            if self._advanced_docking_widget:
                self._advanced_docking_widget.destroy()
                self._advanced_docking_widget = None
                carb.log_info("[KCL Project] Advanced docking UI destroyed")
        except Exception as e:
            carb.log_error(f"[KCL Project] Error destroying advanced UI: {str(e)}")
    
    def _on_clear_scene(self):
        import omni.usd
        import omni.kit.commands
        
        carb.log_info("[KCL Project] Starting aggressive scene clearing...")
        
        # Clear docking markers first
        if hasattr(self, '_docking_controller'):
            self._docking_controller.clear_markers()
        
        # Use Omniverse commands to delete prims
        prims_to_delete = [
            "/World/DragonX",
            "/World/ISS", 
            "/World/Cameras",
            "/World/Environment",
            "/World/ISS/Canadarm2",
            "/World/Materials",
            "/physicsScene"
        ]
        
        for prim_path in prims_to_delete:
            try:
                # Use Omniverse's DeletePrims command - this is more aggressive
                omni.kit.commands.execute('DeletePrims', paths=[prim_path])
                carb.log_info(f"[KCL Project] Deleted: {prim_path}")
            except Exception as e:
                carb.log_warn(f"[KCL Project] Could not delete {prim_path}: {str(e)}")
        
        # Clear sensor references
        if hasattr(self, '_docking_imus'):
            self._docking_imus = {}
            
        if hasattr(self, '_rtx_lidars'):
            self._rtx_lidars = {}
            
        if hasattr(self, '_lidar_visualizations'):
            self._lidar_visualizations = {}

        carb.log_info("[KCL Project] Aggressive scene clearing completed.")

    def on_shutdown(self):
        # Stop docking update loop if active
        self._stop_docking_update_loop()
        
        # Destroy advanced UI if active
        self._destroy_advanced_ui()
        
        if self._widget:
            self._widget.destroy()
            self._widget = None

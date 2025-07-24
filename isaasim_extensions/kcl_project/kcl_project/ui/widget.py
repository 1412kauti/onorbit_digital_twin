# kcl_project/ui/widget.py

import omni.ui as ui
import carb
import omni.kit.app
from .lidar_widget import LidarDisplayWidget

class KclMainWidget:
    def __init__(self, on_load_scene, on_clear_scene):
        self._on_load_scene = on_load_scene
        self._on_clear_scene = on_clear_scene
        self._on_load_visual_inspection_scene = None
        
        self.window = ui.Window("KCL Project Extension", width=300, height=500)
        with self.window.frame:
            with ui.VStack(spacing=15, alignment=ui.Alignment.CENTER):
                ui.Label("KCL Project: Environment Loader", style={"font_size": 18, "color": 0xFF00FF00})
                
                # Docking Operations Selection
                ui.Label("Select Docking Mode", style={"font_size": 16, "color": 0xFF0080FF})
                
                # Docking Operations Dropdown
                with ui.HStack(spacing=10):
                    ui.Label("Mode:", width=50)
                    self._docking_operation_combo = ui.ComboBox(0,
                        "Select Mode...",
                        "Simple Docking",
                        "Advanced Docking",
                        "Visual Inspection",
                        width=150)
                    self._docking_operation_combo.model.add_item_changed_fn(self._on_operation_changed)
                
                # Dynamic operations container
                self._dynamic_operations_frame = ui.Frame()
                
                # Create initial interface (empty until selection)
                self._create_operation_interface()

        # IMU display components
        self._imu_window = None
        self._imu_labels = {}
        self._update_stream = None
        self._get_imu_data_callback = None
        self._connect_sensors_callback = None
        self._sensors_connected = False
        
        # LiDAR display components
        self._lidar_window = None
        self._get_lidar_data_callback = None
        self._setup_lidar_visualization_callback = None
        
        # Docking callbacks
        self._randomize_dragon_callback = None
        self._start_docking_callback = None
        self._stop_docking_callback = None
        self._reset_dragon_callback = None
        
        # Joint control callbacks
        self._joint_control_callback = None
        self._current_joint_control_method = "Joint Control"
        
        # UI update scheduling
        self._pending_ui_update = False

    def set_imu_data_callback(self, callback):
        """Set the callback function to get IMU data."""
        self._get_imu_data_callback = callback


    def set_connect_sensors_callback(self, callback):
        """Set the callback function to connect sensors."""
        self._connect_sensors_callback = callback
    
    def set_lidar_data_callback(self, callback):
        """Set the callback function to get LiDAR data."""
        self._get_lidar_data_callback = callback
    
    def set_lidar_visualization_callback(self, callback):
        """Set the callback function to setup LiDAR visualization."""
        self._setup_lidar_visualization_callback = callback
    
    def set_docking_callbacks(self, randomize_callback, start_callback, stop_callback):
        """Set the callback functions for docking operations."""
        self._randomize_dragon_callback = randomize_callback
        self._start_docking_callback = start_callback
        self._stop_docking_callback = stop_callback
    
    def set_reset_dragon_callback(self, callback):
        """Set the callback function for resetting DragonX position."""
        self._reset_dragon_callback = callback
    
    def set_visual_inspection_callback(self, callback):
        """Set the callback function for loading Visual Inspection scene."""
        self._on_load_visual_inspection_scene = callback
    
    def set_joint_control_callback(self, callback):
        """Set the callback function for joint control operations."""
        self._joint_control_callback = callback

    def _on_connect_sensors(self):
        """FIXED: Connect sensors when explicitly requested."""
        if self._connect_sensors_callback and not self._sensors_connected:
            self._connect_sensors_callback()
            self._sensors_connected = True
            carb.log_info("[Widget] Sensors connected")

    def _on_show_imu_data(self):
        """Show or hide the IMU data display."""
        if not self._sensors_connected:
            carb.log_warn("[Widget] Please connect sensors first")
            return
            
        if self._imu_window is None and self._get_imu_data_callback:
            self._create_imu_display()
        elif self._imu_window:
            self._destroy_imu_display()

    def _on_show_lidar_data(self):
        """Show or hide the LiDAR data display."""
        if not self._sensors_connected:
            carb.log_warn("[Widget] Please connect sensors first")
            return
            
        if self._lidar_window is None and self._get_lidar_data_callback:
            self._create_lidar_display()
        elif self._lidar_window:
            self._destroy_lidar_display()
    
    def _on_randomize_dragon(self):
        """Handle randomize DragonX button click."""
        if self._randomize_dragon_callback:
            success = self._randomize_dragon_callback()
            if success:
                carb.log_info("[Widget] DragonX position randomized")
            else:
                carb.log_error("[Widget] Failed to randomize DragonX position")
        else:
            carb.log_warn("[Widget] No randomize callback set")
    
    def _on_start_docking(self, mode_name=None):
        """Handle start docking button click."""
        if mode_name is None:
            # Legacy support - determine mode from current operation
            selected_operation = self._docking_operation_combo.model.get_item_value_model().get_value_as_int()
            if selected_operation == 1:  # Simple Docking
                mode_name = "Simple"
            elif selected_operation == 2:  # Advanced Docking
                mode_name = "Advanced"
            else:
                carb.log_warn("[Widget] Please select a docking operation first")
                return
        
        if self._start_docking_callback:
            success = self._start_docking_callback(mode_name)
            if success:
                carb.log_info(f"[Widget] Started {mode_name} docking")
            else:
                carb.log_error(f"[Widget] Failed to start {mode_name} docking")
        else:
            carb.log_warn("[Widget] No start docking callback set")
    
    def _on_stop_docking(self):
        """Handle stop docking button click."""
        if self._stop_docking_callback:
            self._stop_docking_callback()
            carb.log_info("[Widget] Docking stopped")
        else:
            carb.log_warn("[Widget] No stop docking callback set")
    
    def _on_reset_dragon(self):
        """Handle reset DragonX button click."""
        if self._reset_dragon_callback:
            success = self._reset_dragon_callback()
            if success:
                carb.log_info("[Widget] DragonX position reset to Load Scene position")
            else:
                carb.log_error("[Widget] Failed to reset DragonX position")
        else:
            carb.log_warn("[Widget] No reset DragonX callback set")
    
    def _on_simple_approach_speed_changed(self, model):
        """Handle approach speed slider changes in Simple Docking mode."""
        try:
            speed = model.get_value_as_float()
            if hasattr(self, '_simple_approach_speed_label') and self._simple_approach_speed_label:
                self._simple_approach_speed_label.text = f"{speed:.3f}"
            
            # TODO: Add callback to extension for updating docking controller
            # if hasattr(self, '_speed_callback') and self._speed_callback:
            #     self._speed_callback('set_approach_speed', speed)
                
            carb.log_info(f"[Simple Docking Widget] Approach speed changed to {speed:.3f}")
        except Exception as e:
            carb.log_error(f"[Simple Docking Widget] Error changing approach speed: {str(e)}")
    
    def _on_simple_final_speed_changed(self, model):
        """Handle final speed slider changes in Simple Docking mode."""
        try:
            speed = model.get_value_as_float()
            if hasattr(self, '_simple_final_speed_label') and self._simple_final_speed_label:
                self._simple_final_speed_label.text = f"{speed:.3f}"
            
            # TODO: Add callback to extension for updating docking controller
            # if hasattr(self, '_speed_callback') and self._speed_callback:
            #     self._speed_callback('set_final_speed', speed)
                
            carb.log_info(f"[Simple Docking Widget] Final speed changed to {speed:.3f}")
        except Exception as e:
            carb.log_error(f"[Simple Docking Widget] Error changing final speed: {str(e)}")
    
    def _on_advanced_approach_speed_changed(self, model):
        """Handle approach speed slider changes in Advanced Docking mode."""
        try:
            speed = model.get_value_as_float()
            if hasattr(self, '_advanced_approach_speed_label') and self._advanced_approach_speed_label:
                self._advanced_approach_speed_label.text = f"{speed:.3f}"
            
            # TODO: Add callback to extension for updating docking controller
            # if hasattr(self, '_advanced_speed_callback') and self._advanced_speed_callback:
            #     self._advanced_speed_callback('set_approach_speed', speed)
                
            carb.log_info(f"[Advanced Docking Widget] Approach speed changed to {speed:.3f}")
        except Exception as e:
            carb.log_error(f"[Advanced Docking Widget] Error changing approach speed: {str(e)}")
    
    def _on_advanced_final_speed_changed(self, model):
        """Handle final speed slider changes in Advanced Docking mode."""
        try:
            speed = model.get_value_as_float()
            if hasattr(self, '_advanced_final_speed_label') and self._advanced_final_speed_label:
                self._advanced_final_speed_label.text = f"{speed:.3f}"
            
            # TODO: Add callback to extension for updating docking controller
            # if hasattr(self, '_advanced_speed_callback') and self._advanced_speed_callback:
            #     self._advanced_speed_callback('set_final_speed', speed)
                
            carb.log_info(f"[Advanced Docking Widget] Final speed changed to {speed:.3f}")
        except Exception as e:
            carb.log_error(f"[Advanced Docking Widget] Error changing final speed: {str(e)}")
    
    def _on_joint_control_changed(self, model, item):
        """Handle joint control method dropdown change."""
        selected_method = self._joint_control_combo.model.get_item_value_model().get_value_as_int()
        method_names = ["Joint Control", "ROS2 Control"]
        
        if selected_method < len(method_names):
            self._current_joint_control_method = method_names[selected_method]
            carb.log_info(f"[Widget] Joint control method changed to: {self._current_joint_control_method}")
            
            # If ROS2 Control is selected, clear scene and load USD
            if self._current_joint_control_method == "ROS2 Control":
                self._handle_ros2_control_selection()
            
            # Notify extension of the change
            if self._joint_control_callback:
                self._joint_control_callback("method_changed", self._current_joint_control_method)
        
        # Defer the UI update to avoid clearing during event callback
        self._schedule_ui_update()
    
    def _on_operation_changed(self, model, item):
        """Handle docking operation dropdown change."""
        self._create_operation_interface()
        
    def _create_operation_interface(self):
        """Create interface based on selected docking operation."""
        selected_operation = self._docking_operation_combo.model.get_item_value_model().get_value_as_int()
        operation_names = [
            "Select Mode...",
            "Simple Docking",
            "Advanced Docking",
            "Visual Inspection"
        ]
        
        operation_name = operation_names[selected_operation] if selected_operation < len(operation_names) else "Unknown"
        
        # Clear existing interface
        self._dynamic_operations_frame.clear()
        
        with self._dynamic_operations_frame:
            with ui.VStack(spacing=10, alignment=ui.Alignment.CENTER):
                if operation_name == "Select Mode...":
                    self._create_default_interface()
                elif operation_name == "Simple Docking":
                    self._create_simple_docking_interface()
                elif operation_name == "Advanced Docking":
                    self._create_advanced_docking_interface()
                elif operation_name == "Visual Inspection":
                    self._create_visual_inspection_interface()
                else:
                    ui.Label("Unknown mode selected", style={"color": 0xFFFF0000})
    
    def _create_default_interface(self):
        """Create default interface when no mode is selected."""
        ui.Label("Select a mode to get started", style={"font_size": 12, "color": 0xFFFFFFFF})
        ui.Label("Available modes:", style={"font_size": 12, "color": 0xFF0080FF})
        
        with ui.VStack(spacing=3):
            ui.Label("• Simple Docking - Basic docking with console output", style={"font_size": 10, "color": 0xFFFFFFFF})
            ui.Label("• Advanced Docking - Real-time monitoring with UI", style={"font_size": 10, "color": 0xFFFFFFFF})
            ui.Label("• Visual Inspection - Camera views and sensor visualization", style={"font_size": 10, "color": 0xFFFFFFFF})
    
    def _create_simple_docking_interface(self):
        """Create interface for Simple docking operation."""
        ui.Label("Simple Docking", style={"font_size": 14, "color": 0xFF00FF00})
        ui.Label("Basic docking with console output", style={"font_size": 12, "color": 0xFFFFFFFF})
        
        ui.Spacer(height=10)
        
        # Scene control buttons
        ui.Label("Scene Controls", style={"font_size": 12, "color": 0xFF0080FF})
        ui.Button("Load Scene", height=40, width=180).set_clicked_fn(self._on_load_scene)
        ui.Button("Clear Scene", height=40, width=180).set_clicked_fn(self._on_clear_scene)
        
        ui.Spacer(height=10)
        
        # Speed Control Section
        ui.Label("Speed Controls", style={"font_size": 12, "color": 0xFF0080FF})
        
        with ui.VStack(spacing=5):
            # Approach Speed Slider
            with ui.HStack(spacing=5):
                ui.Label("Approach Speed:", width=100, style={"font_size": 11})
                self._simple_approach_speed_slider = ui.FloatSlider(
                    min=0.05, max=2.0, step=0.01,
                    model=ui.SimpleFloatModel(0.15),
                    width=140, height=18
                )
                self._simple_approach_speed_label = ui.Label("0.15", width=40, style={"font_size": 11, "color": 0xFF00FF00})
            
            # Final Speed Slider
            with ui.HStack(spacing=5):
                ui.Label("Final Speed:", width=100, style={"font_size": 11})
                self._simple_final_speed_slider = ui.FloatSlider(
                    min=0.02, max=1.0, step=0.01,
                    model=ui.SimpleFloatModel(0.12),
                    width=140, height=18
                )
                self._simple_final_speed_label = ui.Label("0.12", width=40, style={"font_size": 11, "color": 0xFF00FF00})
            
            # Speed control callbacks
            self._simple_approach_speed_slider.model.add_value_changed_fn(self._on_simple_approach_speed_changed)
            self._simple_final_speed_slider.model.add_value_changed_fn(self._on_simple_final_speed_changed)
        
        ui.Spacer(height=10)
        
        # Docking control buttons
        ui.Label("Docking Controls", style={"font_size": 12, "color": 0xFF0080FF})
        with ui.HStack(spacing=10):
            ui.Button("Randomize Position", height=35, width=85).set_clicked_fn(self._on_randomize_dragon)
            ui.Button("Start Docking", height=35, width=85).set_clicked_fn(lambda: self._on_start_docking("Simple"))
        
        ui.Button("Stop Docking", height=35, width=180).set_clicked_fn(self._on_stop_docking)
        ui.Button("Reset DragonX", height=35, width=180).set_clicked_fn(self._on_reset_dragon)
    
    def _create_advanced_docking_interface(self):
        """Create interface for Advanced docking operation."""
        ui.Label("Advanced Docking", style={"font_size": 14, "color": 0xFF00FF00})
        ui.Label("Real-time monitoring with UI updates", style={"font_size": 12, "color": 0xFFFFFFFF})
        
        ui.Spacer(height=10)
        
        # Scene control buttons
        ui.Label("Scene Controls", style={"font_size": 12, "color": 0xFF0080FF})
        ui.Button("Load Scene", height=40, width=180).set_clicked_fn(self._on_load_scene)
        ui.Button("Clear Scene", height=40, width=180).set_clicked_fn(self._on_clear_scene)
        
        ui.Spacer(height=10)
        
        # Connect sensors first for advanced mode
        ui.Label("Sensor Setup", style={"font_size": 12, "color": 0xFF0080FF})
        ui.Button("Connect Sensors", height=35, width=180).set_clicked_fn(self._on_connect_sensors)
        
        ui.Spacer(height=5)
        
        # Sensor data display buttons
        with ui.HStack(spacing=10):
            ui.Button("Show IMU Data", height=35, width=85).set_clicked_fn(self._on_show_imu_data)
            ui.Button("Show LiDAR Data", height=35, width=85).set_clicked_fn(self._on_show_lidar_data)
        
        ui.Spacer(height=10)
        
        # Speed Control Section
        ui.Label("Speed Controls", style={"font_size": 12, "color": 0xFF0080FF})
        
        with ui.VStack(spacing=5):
            # Approach Speed Slider
            with ui.HStack(spacing=5):
                ui.Label("Approach Speed:", width=100, style={"font_size": 11})
                self._advanced_approach_speed_slider = ui.FloatSlider(
                    min=0.05, max=2.0, step=0.01,
                    model=ui.SimpleFloatModel(0.15),
                    width=140, height=18
                )
                self._advanced_approach_speed_label = ui.Label("0.15", width=40, style={"font_size": 11, "color": 0xFF00FF00})
            
            # Final Speed Slider
            with ui.HStack(spacing=5):
                ui.Label("Final Speed:", width=100, style={"font_size": 11})
                self._advanced_final_speed_slider = ui.FloatSlider(
                    min=0.02, max=1.0, step=0.01,
                    model=ui.SimpleFloatModel(0.12),
                    width=140, height=18
                )
                self._advanced_final_speed_label = ui.Label("0.12", width=40, style={"font_size": 11, "color": 0xFF00FF00})
            
            # Speed control callbacks
            self._advanced_approach_speed_slider.model.add_value_changed_fn(self._on_advanced_approach_speed_changed)
            self._advanced_final_speed_slider.model.add_value_changed_fn(self._on_advanced_final_speed_changed)
        
        ui.Spacer(height=10)
        
        # Docking control buttons
        ui.Label("Docking Controls", style={"font_size": 12, "color": 0xFF0080FF})
        with ui.HStack(spacing=10):
            ui.Button("Randomize Position", height=35, width=85).set_clicked_fn(self._on_randomize_dragon)
            ui.Button("Start Advanced Docking", height=35, width=85).set_clicked_fn(lambda: self._on_start_docking("Advanced"))
        
        ui.Button("Stop Docking", height=35, width=180).set_clicked_fn(self._on_stop_docking)
        ui.Button("Reset DragonX", height=35, width=180).set_clicked_fn(self._on_reset_dragon)
        
        ui.Spacer(height=10)
        
        # Advanced mode info
        ui.Label("Advanced Monitoring", style={"font_size": 12, "color": 0xFF0080FF})
        ui.Label("Real-time metrics window opens automatically", style={"font_size": 10, "color": 0xFFFFFFFF})
        ui.Label("Sensor data windows available above", style={"font_size": 10, "color": 0xFFFFFFFF})
    
    def _create_visual_inspection_interface(self):
        """Create interface for Visual Inspection operation."""
        ui.Label("Visual Inspection", style={"font_size": 14, "color": 0xFF00FF00})

        ui.Spacer(height=10)

        # Scene control buttons
        ui.Label("Scene Controls", style={"font_size": 12, "color": 0xFF0080FF})
        ui.Button("Load Visual Inspection Scene", height=40, width=180).set_clicked_fn(self._on_load_visual_inspection_scene)
        ui.Button("Clear Scene", height=40, width=180).set_clicked_fn(self._on_clear_scene)

        ui.Spacer(height=10)
        
        # Joint Control Method Selection for Visual Inspection
        ui.Label("Joint Control Method", style={"font_size": 12, "color": 0xFF0080FF})
        
        # Joint Control Methods Dropdown
        with ui.HStack(spacing=10):
            ui.Label("Method:", width=50, style={"font_size": 11})
            self._joint_control_combo = ui.ComboBox(0,
                "Joint Control",
                "ROS2_Control",
                width=120)
            self._joint_control_combo.model.add_item_changed_fn(self._on_joint_control_changed)
        
        ui.Spacer(height=5)
        
        # Joint Control Section for Visual Inspection
        ui.Label(f"Joint Control ({self._current_joint_control_method})", style={"font_size": 12, "color": 0xFF0080FF})
        
        if self._current_joint_control_method == "Joint Control":
            self._create_direct_control_ui()
        elif self._current_joint_control_method == "ROS2_Control":
            self._create_ros2_control_ui()
    
    def _create_simple_mode_buttons(self):
        """Create buttons for Simple docking mode."""
        ui.Label("Simple Docking", style={"font_size": 14, "color": 0xFF00FF00})
        ui.Label("Basic docking with console output", style={"font_size": 12, "color": 0xFFFFFFFF})
        
        ui.Spacer(height=10)
        
        # Docking control buttons
        with ui.HStack(spacing=10):
            ui.Button("Randomize Position", height=35, width=85).set_clicked_fn(self._on_randomize_dragon)
            ui.Button("Start Docking", height=35, width=85).set_clicked_fn(self._on_start_docking)
        
        ui.Button("Stop Docking", height=35, width=180).set_clicked_fn(self._on_stop_docking)
    
    def _create_advanced_mode_buttons(self):
        """Create buttons for Advanced docking mode."""
        ui.Label("Advanced Docking", style={"font_size": 14, "color": 0xFF00FF00})
        ui.Label("Real-time monitoring with UI updates", style={"font_size": 12, "color": 0xFFFFFFFF})
        
        ui.Spacer(height=10)
        
        # Auto-connect sensors for advanced mode
        ui.Button("Connect Sensors", height=35, width=180).set_clicked_fn(self._on_connect_sensors)
        
        ui.Spacer(height=5)
        
        # Docking control buttons
        with ui.HStack(spacing=10):
            ui.Button("Randomize Position", height=35, width=85).set_clicked_fn(self._on_randomize_dragon)
            ui.Button("Start Advanced Docking", height=35, width=85).set_clicked_fn(self._on_start_docking)
        
        ui.Button("Stop Docking", height=35, width=180).set_clicked_fn(self._on_stop_docking)
        
        ui.Spacer(height=10)
        
        # Advanced mode shows monitoring UI automatically
        ui.Label("Advanced Monitoring", style={"font_size": 12, "color": 0xFF0080FF})
        ui.Label("Real-time metrics window opens automatically", style={"font_size": 10, "color": 0xFFFFFFFF})
    
    def _create_direct_control_ui(self):
        """Create UI for Direct Control with preset joint angles and manual control."""
        ui.Label("Preset Positions:", style={"font_size": 12, "color": 0xFF0080FF})
        with ui.HStack(spacing=5):
            ui.Button("Left 1", height=30, width=85).set_clicked_fn(lambda: self._send_joint_command("preset_angles", [3,-36,32,-38,54,12,-42]))
            ui.Button("Left 2", height=30, width=85).set_clicked_fn(lambda: self._send_joint_command("preset_angles", [28,-41,14,48,19,0,-42]))
            ui.Button("Left 3", height=30, width=85).set_clicked_fn(lambda: self._send_joint_command("preset_angles", [28,-61,17,77,7,24,-58]))
        
        with ui.HStack(spacing=5):
            ui.Button("Right 1", height=30, width=85).set_clicked_fn(lambda: self._send_joint_command("preset_angles", [59,60,165,47,60,-42,-75]))
            ui.Button("Right 2", height=30, width=85).set_clicked_fn(lambda: self._send_joint_command("preset_angles", [30,74,180,47,41,-51,-75]))
            ui.Button("Right 3", height=30, width=85).set_clicked_fn(lambda: self._send_joint_command("preset_angles", [81,74,156,84,51,-146,-42]))
        
        ui.Spacer(height=15)
        
        # Manual Joint Control
        ui.Label("Manual Joint Control:", style={"font_size": 12, "color": 0xFF0080FF})
        
        # Initialize joint angle sliders if not already done
        if not hasattr(self, '_joint_sliders'):
            self._joint_sliders = {}
        
        # Create sliders for each joint
        joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6", "Joint 7"]
        
        for i, joint_name in enumerate(joint_names):
            with ui.HStack(spacing=5, height=25):
                ui.Label(f"{joint_name}:", width=60, style={"font_size": 10})
                
                # Create slider for joint angle (-180 to 180 degrees)
                if i not in self._joint_sliders:
                    self._joint_sliders[i] = ui.FloatSlider(
                        min=-180.0, 
                        max=180.0, 
                        step=1.0,
                        width=120
                    )
                    self._joint_sliders[i].model.set_value(0.0)  # Default to 0 degrees
                
                # Value display and input field
                value_model = self._joint_sliders[i].model
                ui.FloatField(model=value_model, width=50, precision=1)
                ui.Label("°", width=15, style={"font_size": 10})
        
        ui.Spacer(height=10)
        
        # Manual control buttons
        with ui.HStack(spacing=10):
            ui.Button("Apply Angles", height=30, width=85).set_clicked_fn(self._apply_manual_angles)
            ui.Button("Reset to Zero", height=30, width=85).set_clicked_fn(self._reset_joint_sliders)
    
    def _create_ros2_control_ui(self):
        """Create UI for ROS2 Control method."""
        ui.Label("ROS2 Control", style={"font_size": 14, "color": 0xFF00FF00})
        ui.Label("Scene and USD file loaded automatically", style={"font_size": 10, "color": 0xFFFFFFFF})
        ui.Label("Use external ROS2 tools for joint control", style={"font_size": 10, "color": 0xFFFFFFFF})
    
    
    def _apply_manual_angles(self):
        """Apply manually set joint angles to the robot."""
        if not hasattr(self, '_joint_sliders') or not self._joint_sliders:
            carb.log_warn("[Widget] Joint sliders not initialized")
            return
        
        try:
            # Collect angles from all sliders
            joint_angles = []
            for i in range(7):  # 7 joints
                if i in self._joint_sliders:
                    angle = self._joint_sliders[i].model.get_value_as_float()
                    joint_angles.append(angle)
                else:
                    joint_angles.append(0.0)  # Default to 0 if slider not found
            
            # Send the manual angles via the callback
            self._send_joint_command("preset_angles", joint_angles)
            carb.log_info(f"[Widget] Applied manual joint angles: {joint_angles}")
            
        except Exception as e:
            carb.log_error(f"[Widget] Error applying manual angles: {str(e)}")
    
    def _reset_joint_sliders(self):
        """Reset all joint sliders to zero degrees."""
        if not hasattr(self, '_joint_sliders') or not self._joint_sliders:
            carb.log_warn("[Widget] Joint sliders not initialized")
            return
        
        try:
            for i in range(7):  # 7 joints
                if i in self._joint_sliders:
                    self._joint_sliders[i].model.set_value(0.0)
            
            carb.log_info("[Widget] Reset all joint sliders to zero")
            
        except Exception as e:
            carb.log_error(f"[Widget] Error resetting joint sliders: {str(e)}")
    
    def _send_joint_command(self, command_type, command_data):
        """Send joint control command via callback."""
        if self._joint_control_callback:
            self._joint_control_callback(command_type, command_data)
            carb.log_info(f"[Widget] Sent joint command: {command_type} - {command_data}")
        else:
            carb.log_warn("[Widget] No joint control callback set")
    
    def _handle_ros2_control_selection(self):
        """Handle ROS2 Control selection - clear scene and load USD file."""
        try:
            # Clear the current scene
            if self._on_clear_scene:
                self._on_clear_scene()
                carb.log_info("[Widget] Scene cleared for ROS2 Control")
            
            # Load the USD file for ROS2 Control
            # This will be handled by the extension which should have the USD file path
            if self._joint_control_callback:
                self._joint_control_callback("load_ros2_usd", None)
                carb.log_info("[Widget] Requested USD loading for ROS2 Control")
                
        except Exception as e:
            carb.log_error(f"[Widget] Error handling ROS2 Control selection: {str(e)}")
    
    def _schedule_ui_update(self):
        """Schedule a deferred UI update to avoid clearing during event callbacks."""
        if not self._pending_ui_update:
            self._pending_ui_update = True
            # Use asyncio to defer the update to the next event loop cycle
            import asyncio
            
            async def do_update():
                await asyncio.sleep(0.01)  # Small delay to ensure we're out of the event callback
                if self._pending_ui_update:
                    self._pending_ui_update = False
                    # Check if we're in visual inspection mode before updating
                    try:
                        selected_operation = self._docking_operation_combo.model.get_item_value_model().get_value_as_int()
                        if selected_operation == 3:  # Visual Inspection mode
                            self._create_operation_interface()
                    except Exception as e:
                        carb.log_error(f"[Widget] Error in deferred UI update: {str(e)}")
            
            # Schedule the update
            asyncio.ensure_future(do_update())

    def _create_imu_display(self):
        """Create the IMU data display window."""
        self._imu_window = ui.Window("IMU Sensor Data", width=400, height=600)
        with self._imu_window.frame:
            with ui.VStack(spacing=10):
                ui.Label("IMU Sensor Readings", style={"font_size": 18, "color": 0xFF00FF00})

                # Create labels for each IMU sensor
                self._imu_labels = {}

                # DragonX IMUs
                ui.Label("DragonX Sensors", style={"font_size": 14, "color": 0xFF0080FF})
                self._imu_labels["dragon_primary_imu"] = self._create_imu_sensor_display("Dragon Primary IMU")
                self._imu_labels["dragon_docking_imu"] = self._create_imu_sensor_display("Dragon Docking IMU")

                ui.Separator()

                # ISS IMUs
                ui.Label("ISS Sensors", style={"font_size": 14, "color": 0xFF0080FF})
                self._imu_labels["iss_docking_port_imu"] = self._create_imu_sensor_display("ISS Docking Port IMU")
                self._imu_labels["iss_structure_imu"] = self._create_imu_sensor_display("ISS Structure IMU")

        # Start update timer
        self._update_stream = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._update_imu_display, name="imu_display_update"
        )

    def _create_imu_sensor_display(self, name):
        """Create UI elements for a single IMU sensor."""
        labels = {}
        with ui.CollapsableFrame(name, collapsed=False):
            with ui.VStack(spacing=5):
                labels["lin_acc"] = ui.Label("Linear Acceleration: (0, 0, 0)")
                labels["ang_vel"] = ui.Label("Angular Velocity: (0, 0, 0)")
                labels["orientation"] = ui.Label("Orientation: (1, 0, 0, 0)")
                labels["lin_acc_mag"] = ui.Label("Acceleration Magnitude: 0.000")
                labels["ang_vel_mag"] = ui.Label("Angular Velocity Magnitude: 0.000")
                labels["contact"] = ui.Label("Contact Detected: False")
                labels["motion"] = ui.Label("Motion Detected: False")
        return labels

    def _update_imu_display(self, dt):
        """Update the IMU display with current data."""
        if not self._get_imu_data_callback or not self._imu_window:
            return

        try:
            imu_data = self._get_imu_data_callback()
            for imu_name, data in imu_data.items():
                if imu_name in self._imu_labels:
                    labels = self._imu_labels[imu_name]

                    # Update labels with current data
                    labels["lin_acc"].text = f"Linear Acceleration: {data['linear_acceleration']}"
                    labels["ang_vel"].text = f"Angular Velocity: {data['angular_velocity']}"
                    labels["orientation"].text = f"Orientation: {data['orientation']}"
                    labels["lin_acc_mag"].text = f"Acceleration Magnitude: {data['linear_acceleration_magnitude']:.3f}"
                    labels["ang_vel_mag"].text = f"Angular Velocity Magnitude: {data['angular_velocity_magnitude']:.3f}"

                    # Color-code contact and motion detection
                    contact_color = 0xFF00FF00 if data['contact_detected'] else 0xFFFFFFFF
                    motion_color = 0xFF00FF00 if data['motion_detected'] else 0xFFFFFFFF

                    labels["contact"].text = f"Contact Detected: {data['contact_detected']}"
                    labels["contact"].style = {"color": contact_color}

                    labels["motion"].text = f"Motion Detected: {data['motion_detected']}"
                    labels["motion"].style = {"color": motion_color}

        except Exception as e:
            carb.log_error(f"[IMU Widget] Error updating display: {str(e)}")

    def _destroy_imu_display(self):
        """Destroy the IMU display window and cleanup."""
        if self._update_stream:
            self._update_stream.unsubscribe()
            self._update_stream = None

        if self._imu_window:
            self._imu_window.destroy()
            self._imu_window = None

        self._imu_labels = {}
    
    def _create_lidar_display(self):
        """Create the LiDAR data display window."""
        if self._get_lidar_data_callback and self._setup_lidar_visualization_callback:
            self._lidar_window = LidarDisplayWidget(
                self._get_lidar_data_callback,
                self._setup_lidar_visualization_callback
            )
            carb.log_info("[Widget] LiDAR display created")
        else:
            carb.log_warn("[Widget] LiDAR callbacks not set, creating display without visualization")
            # Create display with limited functionality if visualization callback is not available
            if self._get_lidar_data_callback:
                self._lidar_window = LidarDisplayWidget(
                    self._get_lidar_data_callback,
                    None  # No visualization callback
                )
                carb.log_info("[Widget] LiDAR display created with limited functionality")
    
    def _destroy_lidar_display(self):
        """Destroy the LiDAR display window and cleanup."""
        if self._lidar_window:
            self._lidar_window.destroy()
            self._lidar_window = None
            carb.log_info("[Widget] LiDAR display destroyed")

    def destroy(self):
        """Cleanup all UI components."""
        # Destroy IMU display first
        self._destroy_imu_display()
        
        # Destroy LiDAR display
        self._destroy_lidar_display()

        # Destroy main window
        if self.window:
            self.window.destroy()
            self.window = None

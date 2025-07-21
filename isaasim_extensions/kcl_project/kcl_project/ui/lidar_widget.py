# kcl_project/ui/lidar_widget.py

import omni.ui as ui
import carb
import omni.kit.app

class LidarDisplayWidget:
    def __init__(self, get_lidar_data_callback, setup_visualization_callback):
        self.get_lidar_data = get_lidar_data_callback
        self.setup_visualization = setup_visualization_callback
        self.window = ui.Window("LiDAR Sensor Data", width=500, height=700)
        self._build_ui()
        
        # Update timer
        self._update_stream = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._update_display, name="lidar_display_update"
        )
        
        # Track visualization state
        self._visualization_enabled = {}
    
    def _build_ui(self):
        with self.window.frame:
            with ui.VStack(spacing=10):
                ui.Label("LiDAR Sensor Readings", style={"font_size": 18, "color": 0xFF00FF00})
                
                # Control buttons
                with ui.HStack(spacing=10):
                    enable_all_btn = ui.Button("Enable All Visualizations", height=35, width=150)
                    disable_all_btn = ui.Button("Disable All Visualizations", height=35, width=150)
                    
                    if self.setup_visualization:
                        enable_all_btn.set_clicked_fn(self._enable_all_visualizations)
                        disable_all_btn.set_clicked_fn(self._disable_all_visualizations)
                    else:
                        enable_all_btn.enabled = False
                        disable_all_btn.enabled = False
                        enable_all_btn.text = "Visualization Unavailable"
                        disable_all_btn.text = "Visualization Unavailable"
                
                ui.Separator()
                
                # Create displays for each LiDAR sensor
                self._lidar_labels = {}
                self._visualization_buttons = {}
                
                # DragonX LiDAR sensors
                ui.Label("DragonX LiDAR Sensors", style={"font_size": 14, "color": 0xFF0080FF})
                self._lidar_labels["dragon_navigation_lidar"] = self._create_lidar_display("Dragon Navigation LiDAR", "dragon_navigation_lidar")
                self._lidar_labels["dragon_docking_lidar"] = self._create_lidar_display("Dragon Docking LiDAR", "dragon_docking_lidar")
                
                ui.Separator()
                
                # ISS LiDAR sensors
                ui.Label("ISS LiDAR Sensors", style={"font_size": 14, "color": 0xFF0080FF})
                self._lidar_labels["iss_approach_lidar"] = self._create_lidar_display("ISS Approach LiDAR", "iss_approach_lidar")
                
                ui.Separator()
                
                # Environment analysis
                ui.Label("Environment Analysis", style={"font_size": 14, "color": 0xFFFF8000})
                self._environment_labels = self._create_environment_display()
    
    def _create_lidar_display(self, name, sensor_id):
        """Create UI elements for a single LiDAR sensor."""
        labels = {}
        
        with ui.CollapsableFrame(name, collapsed=False):
            with ui.VStack(spacing=5):
                # Sensor status and basic info
                labels["status"] = ui.Label("Status: Inactive")
                labels["point_count"] = ui.Label("Point Count: 0")
                labels["range_min"] = ui.Label("Min Range: 0.0 m")
                labels["range_max"] = ui.Label("Max Range: 0.0 m")
                labels["fov_horizontal"] = ui.Label("Horizontal FOV: 0.0°")
                labels["fov_vertical"] = ui.Label("Vertical FOV: 0.0°")
                
                # Visualization control
                with ui.HStack(spacing=10):
                    viz_button = ui.Button("Enable Visualization", height=30, width=140)
                    if self.setup_visualization:
                        viz_button.set_clicked_fn(lambda sensor=sensor_id: self._toggle_visualization(sensor))
                    else:
                        viz_button.enabled = False
                        viz_button.text = "Visualization Unavailable"
                    self._visualization_buttons[sensor_id] = viz_button
                    
                    viz_status = "Visualization: OFF" if self.setup_visualization else "Visualization: N/A"
                    labels["visualization_status"] = ui.Label(viz_status)
                    
                ui.Separator()
        
        return labels
    
    def _create_environment_display(self):
        """Create UI elements for environment analysis."""
        labels = {}
        
        with ui.CollapsableFrame("Environment Analysis", collapsed=False):
            with ui.VStack(spacing=5):
                labels["sensor_count"] = ui.Label("Active Sensors: 0")
                labels["objects_detected"] = ui.Label("Objects Detected: 0")
                labels["closest_distance"] = ui.Label("Closest Object: ∞ m")
                labels["obstacle_warning"] = ui.Label("Obstacle Warning: None")
                labels["navigation_clear"] = ui.Label("Navigation Clear: Unknown")
                
                ui.Separator()
                
                # Object detection details
                ui.Label("Detection Details:", style={"font_size": 12, "color": 0xFFFFFF80})
                labels["detection_details"] = ui.Label("No objects detected")
        
        return labels
    
    def _toggle_visualization(self, sensor_id):
        """Toggle visualization for a specific sensor."""
        try:
            if not self.setup_visualization:
                carb.log_warn(f"[LiDAR Widget] Visualization not available - setup callback not provided")
                return
            
            if sensor_id not in self._visualization_enabled:
                self._visualization_enabled[sensor_id] = False
            
            # Toggle state
            self._visualization_enabled[sensor_id] = not self._visualization_enabled[sensor_id]
            
            if self._visualization_enabled[sensor_id]:
                # Enable visualization
                success = self.setup_visualization(sensor_id)
                if success:
                    self._visualization_buttons[sensor_id].text = "Disable Visualization"
                    if sensor_id in self._lidar_labels:
                        self._lidar_labels[sensor_id]["visualization_status"].text = "Visualization: ON"
                        self._lidar_labels[sensor_id]["visualization_status"].style = {"color": 0xFF00FF00}
                    carb.log_info(f"[LiDAR Widget] Enabled visualization for {sensor_id}")
                else:
                    self._visualization_enabled[sensor_id] = False
                    carb.log_warn(f"[LiDAR Widget] Failed to enable visualization for {sensor_id}")
            else:
                # Disable visualization
                self._visualization_buttons[sensor_id].text = "Enable Visualization"
                if sensor_id in self._lidar_labels:
                    self._lidar_labels[sensor_id]["visualization_status"].text = "Visualization: OFF"
                    self._lidar_labels[sensor_id]["visualization_status"].style = {"color": 0xFFFFFFFF}
                carb.log_info(f"[LiDAR Widget] Disabled visualization for {sensor_id}")
                
        except Exception as e:
            carb.log_error(f"[LiDAR Widget] Error toggling visualization for {sensor_id}: {str(e)}")
    
    def _enable_all_visualizations(self):
        """Enable visualization for all sensors."""
        for sensor_id in ["dragon_navigation_lidar", "dragon_docking_lidar", "iss_approach_lidar"]:
            if sensor_id in self._visualization_buttons and not self._visualization_enabled.get(sensor_id, False):
                self._toggle_visualization(sensor_id)
    
    def _disable_all_visualizations(self):
        """Disable visualization for all sensors."""
        for sensor_id in ["dragon_navigation_lidar", "dragon_docking_lidar", "iss_approach_lidar"]:
            if sensor_id in self._visualization_buttons and self._visualization_enabled.get(sensor_id, False):
                self._toggle_visualization(sensor_id)
    
    def _update_display(self, dt):
        """Update the LiDAR display with current data."""
        if not self.get_lidar_data or not self.window:
            return
        
        try:
            # Get LiDAR data
            lidar_data = self.get_lidar_data()
            
            # Update individual sensor displays
            for sensor_id, data in lidar_data.items():
                if sensor_id in self._lidar_labels:
                    labels = self._lidar_labels[sensor_id]
                    
                    # Update sensor data
                    status_color = 0xFF00FF00 if data.get("status") == "active" else 0xFFFF0000
                    labels["status"].text = f"Status: {data.get('status', 'Unknown')}"
                    labels["status"].style = {"color": status_color}
                    
                    labels["point_count"].text = f"Point Count: {data.get('point_count', 0)}"
                    labels["range_min"].text = f"Min Range: {data.get('range_min', 0.0):.2f} m"
                    labels["range_max"].text = f"Max Range: {data.get('range_max', 0.0):.2f} m"
                    labels["fov_horizontal"].text = f"Horizontal FOV: {data.get('fov_horizontal', 0.0):.1f}°"
                    labels["fov_vertical"].text = f"Vertical FOV: {data.get('fov_vertical', 0.0):.1f}°"
            
            # Get environment analysis
            environment_data = self.get_lidar_data(analysis_mode=True)
            if environment_data and self._environment_labels:
                env_labels = self._environment_labels
                
                sensor_count = environment_data.get("sensor_count", 0)
                objects_detected = len(environment_data.get("objects_detected", []))
                closest_distance = environment_data.get("closest_distance", float('inf'))
                obstacle_warning = environment_data.get("obstacle_warning", False)
                navigation_clear = environment_data.get("navigation_clear", True)
                
                env_labels["sensor_count"].text = f"Active Sensors: {sensor_count}"
                env_labels["objects_detected"].text = f"Objects Detected: {objects_detected}"
                
                # Format closest distance
                if closest_distance == float('inf'):
                    distance_text = "Closest Object: No objects detected"
                else:
                    distance_text = f"Closest Object: {closest_distance:.2f} m"
                env_labels["closest_distance"].text = distance_text
                
                # Color-code warnings
                warning_color = 0xFFFF0000 if obstacle_warning else 0xFF00FF00
                warning_text = "OBSTACLE DETECTED" if obstacle_warning else "Clear"
                env_labels["obstacle_warning"].text = f"Obstacle Warning: {warning_text}"
                env_labels["obstacle_warning"].style = {"color": warning_color}
                
                nav_color = 0xFF00FF00 if navigation_clear else 0xFFFF0000
                nav_text = "CLEAR" if navigation_clear else "BLOCKED"
                env_labels["navigation_clear"].text = f"Navigation Clear: {nav_text}"
                env_labels["navigation_clear"].style = {"color": nav_color}
                
                # Update detection details
                detection_details = []
                for obj in environment_data.get("objects_detected", []):
                    sensor_name = obj.get("sensor", "Unknown")
                    point_count = obj.get("point_count", 0)
                    min_range = obj.get("min_range", 0.0)
                    detection_details.append(f"{sensor_name}: {point_count} points, {min_range:.2f}m")
                
                if detection_details:
                    env_labels["detection_details"].text = "\n".join(detection_details)
                else:
                    env_labels["detection_details"].text = "No objects detected"
                
        except Exception as e:
            carb.log_error(f"[LiDAR Widget] Error updating display: {str(e)}")
    
    def destroy(self):
        """Cleanup the widget."""
        if self._update_stream:
            self._update_stream.unsubscribe()
            self._update_stream = None
        
        if self.window:
            self.window.destroy()
            self.window = None
        
        self._lidar_labels = {}
        self._visualization_buttons = {}
        self._environment_labels = {}
        self._visualization_enabled = {}

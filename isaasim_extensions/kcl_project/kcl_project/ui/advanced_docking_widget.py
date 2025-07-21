# kcl_project/ui/advanced_docking_widget.py

import omni.ui as ui
import carb
import omni.kit.app

class AdvancedDockingWidget:
    """UI widget for displaying advanced docking metrics in real-time."""
    
    def __init__(self):
        self.window = None
        self.labels = {}
        self.progress_bar = None
        self.update_stream = None
        self.is_active = False
        self.imu_visualizations = {}
        self.lidar_visualizations = {}
        self.sensor_callbacks = {}
        
    def create_window(self):
        """Create the advanced docking status window."""
        if self.window:
            return
            
        self.window = ui.Window("Advanced Docking Status", width=450, height=700)
        with self.window.frame:
            with ui.VStack(spacing=10):
                # Header
                ui.Label("Advanced Docking Monitor", style={"font_size": 18, "color": 0xFF00FF00})
                
                # Status indicators
                with ui.HStack(spacing=20):
                    self.labels["phase"] = ui.Label("Phase: Inactive", style={"font_size": 14, "color": 0xFF0080FF})
                    self.labels["active"] = ui.Label("Status: Stopped", style={"font_size": 14, "color": 0xFFFF0000})
                
                ui.Separator()
                
                # Progress section
                ui.Label("Progress", style={"font_size": 16, "color": 0xFF0080FF})
                self.progress_bar = ui.ProgressBar(width=400, height=20)
                self.labels["progress"] = ui.Label("0.0%", style={"font_size": 12})
                
                ui.Separator()
                
                # Real-time position section
                ui.Label("Real-Time Position", style={"font_size": 16, "color": 0xFF0080FF})
                with ui.VStack(spacing=3):
                    self.labels["current_position"] = ui.Label("Current Position: (0, 0, 0)", style={"font_size": 12, "color": 0xFF00FF00})
                    self.labels["target_position"] = ui.Label("Target Position: (0, 0, 0)", style={"font_size": 12, "color": 0xFFFFFF00})
                
                ui.Separator()
                
                # Metrics section
                ui.Label("Docking Metrics", style={"font_size": 16, "color": 0xFF0080FF})
                
                with ui.VStack(spacing=5):
                    self.labels["distance"] = ui.Label("Distance to Target: 0.0 units")
                    self.labels["velocity"] = ui.Label("Approach Velocity: 0.0 units/s")
                    self.labels["alignment_error"] = ui.Label("Alignment Error: 0.0 units")
                    self.labels["time_remaining"] = ui.Label("Time Remaining: 0.0 s")
                    self.labels["speed_setting"] = ui.Label("Speed Setting: 0.0")
                
                ui.Separator()
                
                # Sensor Visualization section
                ui.Label("Sensor Visualization", style={"font_size": 16, "color": 0xFF0080FF})
                
                with ui.VStack(spacing=5):
                    # IMU visualization buttons
                    with ui.CollapsableFrame("IMU Sensors", collapsed=False):
                        with ui.VStack(spacing=3):
                            ui.Label("IMU Status:", style={"font_size": 12, "color": 0xFFFFFFFF})
                            
                            with ui.HStack(spacing=5):
                                ui.Button("DragonX IMU", height=25, width=120).set_clicked_fn(lambda: self.toggle_imu_visualization("dragon_primary_imu"))
                                self.labels["dragon_imu_status"] = ui.Label("●", style={"font_size": 14, "color": 0xFFFF0000})
                            
                            with ui.HStack(spacing=5):
                                ui.Button("ISS IMU", height=25, width=120).set_clicked_fn(lambda: self.toggle_imu_visualization("iss_docking_port_imu"))
                                self.labels["iss_imu_status"] = ui.Label("●", style={"font_size": 14, "color": 0xFFFF0000})
                    
                    # LiDAR visualization buttons
                    with ui.CollapsableFrame("LiDAR Sensors", collapsed=False):
                        with ui.VStack(spacing=3):
                            ui.Label("LiDAR Visualization:", style={"font_size": 12, "color": 0xFFFFFFFF})
                            
                            with ui.HStack(spacing=5):
                                ui.Button("Navigation LiDAR", height=25, width=120).set_clicked_fn(lambda: self.toggle_lidar_visualization("dragon_navigation_lidar"))
                                self.labels["nav_lidar_status"] = ui.Label("●", style={"font_size": 14, "color": 0xFFFF0000})
                            
                            with ui.HStack(spacing=5):
                                ui.Button("Docking LiDAR", height=25, width=120).set_clicked_fn(lambda: self.toggle_lidar_visualization("dragon_docking_lidar"))
                                self.labels["dock_lidar_status"] = ui.Label("●", style={"font_size": 14, "color": 0xFFFF0000})
                            
                            with ui.HStack(spacing=5):
                                ui.Button("ISS LiDAR", height=25, width=120).set_clicked_fn(lambda: self.toggle_lidar_visualization("iss_approach_lidar"))
                                self.labels["iss_lidar_status"] = ui.Label("●", style={"font_size": 14, "color": 0xFFFF0000})
                
                ui.Separator()
                
                # Detailed metrics (collapsible)
                with ui.CollapsableFrame("Detailed Metrics", collapsed=True):
                    with ui.VStack(spacing=3):
                        self.labels["last_update"] = ui.Label("Last Update: Never")
                        self.labels["phase_progress"] = ui.Label("Phase Progress: 0.0%")
                        self.labels["velocity_smoothed"] = ui.Label("Smoothed Velocity: 0.0 units/s")
                
                ui.Separator()
                
                # Speed Control Section
                ui.Label("Speed Controls", style={"font_size": 16, "color": 0xFF0080FF})
                
                with ui.VStack(spacing=5):
                    # Approach Speed Slider
                    with ui.HStack(spacing=5):
                        ui.Label("Approach Speed:", width=120, style={"font_size": 12})
                        self.approach_speed_slider = ui.FloatSlider(
                            min=0.05, max=2.0, step=0.01,
                            model=ui.SimpleFloatModel(0.15),
                            width=200, height=20
                        )
                        self.labels["approach_speed_value"] = ui.Label("0.15", style={"font_size": 12, "color": 0xFF00FF00})
                    
                    # Final Speed Slider
                    with ui.HStack(spacing=5):
                        ui.Label("Final Speed:", width=120, style={"font_size": 12})
                        self.final_speed_slider = ui.FloatSlider(
                            min=0.02, max=1.0, step=0.01,
                            model=ui.SimpleFloatModel(0.12),
                            width=200, height=20
                        )
                        self.labels["final_speed_value"] = ui.Label("0.12", style={"font_size": 12, "color": 0xFF00FF00})
                    
                    # Speed control callbacks
                    self.approach_speed_slider.model.add_value_changed_fn(self.on_approach_speed_changed)
                    self.final_speed_slider.model.add_value_changed_fn(self.on_final_speed_changed)
                
                ui.Separator()
                
                # Control buttons
                with ui.HStack(spacing=10):
                    ui.Button("Close", height=30, width=100).set_clicked_fn(self.close_window)
                    ui.Button("Reset", height=30, width=100).set_clicked_fn(self.reset_display)
        
        carb.log_info("[Advanced Docking Widget] Window created")
    
    def close_window(self):
        """Close the advanced docking window."""
        if self.window:
            self.window.destroy()
            self.window = None
        self.is_active = False
        carb.log_info("[Advanced Docking Widget] Window closed")
    
    def reset_display(self):
        """Reset all display values to default."""
        self.update_display({
            "phase": "inactive",
            "active": False,
            "progress": 0.0,
            "distance": 0.0,
            "velocity": 0.0,
            "alignment_error": 0.0,
            "time_remaining": 0.0,
            "speed_setting": 0.0,
            "metrics": {
                "last_update_time": 0.0,
                "phase_progress": 0.0,
                "approach_velocity": 0.0,
                "distance_to_target": 0.0,
                "alignment_error": 0.0,
                "estimated_time_remaining": 0.0
            }
        })
        carb.log_info("[Advanced Docking Widget] Display reset")
    
    def update_display(self, data):
        """Update the display with new docking data."""
        if not self.window or not self.labels:
            return
            
        try:
            # Update phase and status
            phase = data.get("phase", "inactive").title()
            active = data.get("active", False)
            
            self.labels["phase"].text = f"Phase: {phase}"
            self.labels["active"].text = f"Status: {'Active' if active else 'Stopped'}"
            
            # Color-code status
            phase_color = 0xFF00FF00 if active else 0xFF0080FF
            status_color = 0xFF00FF00 if active else 0xFFFF0000
            
            self.labels["phase"].style = {"font_size": 14, "color": phase_color}
            self.labels["active"].style = {"font_size": 14, "color": status_color}
            
            # Update progress
            progress = data.get("progress", 0.0)
            if self.progress_bar:
                self.progress_bar.model.set_value(progress / 100.0)
            self.labels["progress"].text = f"{progress:.1f}%"
            
            # Update real-time position displays
            current_pos = data.get("current_position", (0, 0, 0))
            target_pos = data.get("target_position", (0, 0, 0))
            
            self.labels["current_position"].text = f"Current Position: ({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f})"
            self.labels["target_position"].text = f"Target Position: ({target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f})"
            
            # Update main metrics
            self.labels["distance"].text = f"Distance to Target: {data.get('distance', 0.0):.2f} units"
            self.labels["velocity"].text = f"Approach Velocity: {data.get('velocity', 0.0):.3f} units/s"
            self.labels["alignment_error"].text = f"Alignment Error: {data.get('alignment_error', 0.0):.3f} units"
            self.labels["time_remaining"].text = f"Time Remaining: {data.get('time_remaining', 0.0):.1f} s"
            self.labels["speed_setting"].text = f"Speed Setting: {data.get('speed_setting', 0.0):.3f}"
            
            # Update detailed metrics if available
            metrics = data.get("metrics", {})
            if metrics:
                import time
                last_update = metrics.get("last_update_time", 0.0)
                if last_update > 0:
                    time_str = time.strftime("%H:%M:%S", time.localtime(last_update))
                    self.labels["last_update"].text = f"Last Update: {time_str}"
                
                self.labels["phase_progress"].text = f"Phase Progress: {metrics.get('phase_progress', 0.0) * 100:.1f}%"
                self.labels["velocity_smoothed"].text = f"Smoothed Velocity: {metrics.get('approach_velocity', 0.0):.3f} units/s"
            
            # Color-code critical metrics
            distance = data.get("distance", 0.0)
            velocity = data.get("velocity", 0.0)
            alignment_error = data.get("alignment_error", 0.0)
            
            # Distance: Green if < 5, Yellow if < 20, Red if >= 20
            distance_color = 0xFF00FF00 if distance < 5 else 0xFFFFFF00 if distance < 20 else 0xFFFF0000
            self.labels["distance"].style = {"color": distance_color}
            
            # Velocity: Green if reasonable, Red if too fast
            velocity_color = 0xFF00FF00 if velocity < 1.0 else 0xFFFFFF00 if velocity < 2.0 else 0xFFFF0000
            self.labels["velocity"].style = {"color": velocity_color}
            
            # Alignment error: Green if < 1, Yellow if < 3, Red if >= 3
            alignment_color = 0xFF00FF00 if alignment_error < 1.0 else 0xFFFFFF00 if alignment_error < 3.0 else 0xFFFF0000
            self.labels["alignment_error"].style = {"color": alignment_color}
            
        except Exception as e:
            carb.log_error(f"[Advanced Docking Widget] Error updating display: {str(e)}")
    
    def toggle_imu_visualization(self, imu_id):
        """Toggle IMU visualization for a specific sensor."""
        try:
            # Check if IMU is currently visualized
            if imu_id in self.imu_visualizations:
                # Turn off visualization
                self.imu_visualizations.pop(imu_id)
                status_color = 0xFFFF0000  # Red - off
                carb.log_info(f"[Advanced Docking Widget] IMU visualization disabled for {imu_id}")
            else:
                # Turn on visualization
                self.imu_visualizations[imu_id] = True
                status_color = 0xFF00FF00  # Green - on
                carb.log_info(f"[Advanced Docking Widget] IMU visualization enabled for {imu_id}")
                
                # Call extension callback if available
                if hasattr(self, 'extension_callback') and self.extension_callback:
                    self.extension_callback('setup_imu_visualization', imu_id)
            
            # Update status indicator
            if imu_id == "dragon_primary_imu" and "dragon_imu_status" in self.labels:
                self.labels["dragon_imu_status"].style = {"font_size": 14, "color": status_color}
            elif imu_id == "iss_docking_port_imu" and "iss_imu_status" in self.labels:
                self.labels["iss_imu_status"].style = {"font_size": 14, "color": status_color}
                
        except Exception as e:
            carb.log_error(f"[Advanced Docking Widget] Error toggling IMU visualization: {str(e)}")
    
    def toggle_lidar_visualization(self, lidar_id):
        """Toggle LiDAR visualization for a specific sensor."""
        try:
            # Check if LiDAR is currently visualized
            if lidar_id in self.lidar_visualizations:
                # Turn off visualization
                self.lidar_visualizations.pop(lidar_id)
                status_color = 0xFFFF0000  # Red - off
                carb.log_info(f"[Advanced Docking Widget] LiDAR visualization disabled for {lidar_id}")
            else:
                # Turn on visualization
                self.lidar_visualizations[lidar_id] = True
                status_color = 0xFF00FF00  # Green - on
                carb.log_info(f"[Advanced Docking Widget] LiDAR visualization enabled for {lidar_id}")
                
                # Call extension callback if available
                if hasattr(self, 'extension_callback') and self.extension_callback:
                    self.extension_callback('setup_lidar_visualization', lidar_id)
            
            # Update status indicator
            if lidar_id == "dragon_navigation_lidar" and "nav_lidar_status" in self.labels:
                self.labels["nav_lidar_status"].style = {"font_size": 14, "color": status_color}
            elif lidar_id == "dragon_docking_lidar" and "dock_lidar_status" in self.labels:
                self.labels["dock_lidar_status"].style = {"font_size": 14, "color": status_color}
            elif lidar_id == "iss_approach_lidar" and "iss_lidar_status" in self.labels:
                self.labels["iss_lidar_status"].style = {"font_size": 14, "color": status_color}
                
        except Exception as e:
            carb.log_error(f"[Advanced Docking Widget] Error toggling LiDAR visualization: {str(e)}")
    
    def on_approach_speed_changed(self, model):
        """Handle approach speed slider changes."""
        try:
            speed = model.get_value_as_float()
            self.labels["approach_speed_value"].text = f"{speed:.3f}"
            
            # Call extension callback to update docking controller
            if hasattr(self, 'extension_callback') and self.extension_callback:
                self.extension_callback('set_approach_speed', speed)
                
            carb.log_info(f"[Advanced Docking Widget] Approach speed changed to {speed:.3f}")
        except Exception as e:
            carb.log_error(f"[Advanced Docking Widget] Error changing approach speed: {str(e)}")
    
    def on_final_speed_changed(self, model):
        """Handle final speed slider changes."""
        try:
            speed = model.get_value_as_float()
            self.labels["final_speed_value"].text = f"{speed:.3f}"
            
            # Call extension callback to update docking controller
            if hasattr(self, 'extension_callback') and self.extension_callback:
                self.extension_callback('set_final_speed', speed)
                
            carb.log_info(f"[Advanced Docking Widget] Final speed changed to {speed:.3f}")
        except Exception as e:
            carb.log_error(f"[Advanced Docking Widget] Error changing final speed: {str(e)}")
    
    def update_speed_sliders(self, approach_speed, final_speed):
        """Update speed slider values programmatically."""
        try:
            if hasattr(self, 'approach_speed_slider') and self.approach_speed_slider:
                self.approach_speed_slider.model.set_value(approach_speed)
                self.labels["approach_speed_value"].text = f"{approach_speed:.3f}"
            
            if hasattr(self, 'final_speed_slider') and self.final_speed_slider:
                self.final_speed_slider.model.set_value(final_speed)
                self.labels["final_speed_value"].text = f"{final_speed:.3f}"
                
        except Exception as e:
            carb.log_error(f"[Advanced Docking Widget] Error updating speed sliders: {str(e)}")
    
    def set_extension_callback(self, callback):
        """Set callback to communicate with the extension for sensor operations."""
        self.extension_callback = callback
    
    def show(self):
        """Show the advanced docking window."""
        if not self.window:
            self.create_window()
        self.is_active = True
        carb.log_info("[Advanced Docking Widget] Window shown")
    
    def hide(self):
        """Hide the advanced docking window."""
        if self.window:
            self.window.visible = False
        self.is_active = False
        carb.log_info("[Advanced Docking Widget] Window hidden")
    
    def destroy(self):
        """Destroy the widget and cleanup resources."""
        if self.update_stream:
            self.update_stream.unsubscribe()
            self.update_stream = None
        
        if self.window:
            self.window.destroy()
            self.window = None
        
        self.labels = {}
        self.progress_bar = None
        self.is_active = False
        carb.log_info("[Advanced Docking Widget] Destroyed")

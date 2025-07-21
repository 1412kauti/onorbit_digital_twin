# kcl_project/ui/imu_widget.py

import omni.ui as ui
import carb

class IMUDisplayWidget:
    def __init__(self, get_imu_data_callback):
        self.get_imu_data = get_imu_data_callback
        self.window = ui.Window("IMU Sensor Data", width=400, height=600)
        self._build_ui()
        
        # Update timer
        import omni.kit.app
        self._update_stream = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._update_display, name="imu_display_update"
        )
    
    def _build_ui(self):
        with self.window.frame:
            with ui.VStack(spacing=10):
                ui.Label("IMU Sensor Readings", style={"font_size": 18, "color": 0xFF00FF00})
                
                # Create labels for each IMU sensor
                self._imu_labels = {}
                
                # DragonX IMUs
                ui.Label("DragonX Sensors", style={"font_size": 14, "color": 0xFF0080FF})
                self._imu_labels["dragon_primary_imu"] = self._create_imu_display("Dragon Primary IMU")
                self._imu_labels["dragon_docking_imu"] = self._create_imu_display("Dragon Docking IMU")
                
                ui.Separator()
                
                # ISS IMUs
                ui.Label("ISS Sensors", style={"font_size": 14, "color": 0xFF0080FF})
                self._imu_labels["iss_docking_port_imu"] = self._create_imu_display("ISS Docking Port IMU")
                self._imu_labels["iss_structure_imu"] = self._create_imu_display("ISS Structure IMU")
    
    def _create_imu_display(self, name):
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
    
    def _update_display(self):
        """Update the IMU display with current data."""
        try:
            imu_data = self.get_imu_data()
            
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
    
    def destroy(self):
        if self._update_stream:
            self._update_stream.unsubscribe()
        if self.window:
            self.window.destroy()
            self.window = None

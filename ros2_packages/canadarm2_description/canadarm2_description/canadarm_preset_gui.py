#!/usr/bin/env python3

import tkinter as tk
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import math
import sys

class CanadarmPresetGUI(Node):
    def __init__(self):
        super().__init__('canadarm_preset_gui')
        
        # Define your preset positions in degrees
        self.presets = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'stow': [0.0, -90.0, 90.0, 0.0, -90.0, 0.0],
            'deploy': [45.0, -45.0, 45.0, 0.0, -45.0, 90.0],
            'inspect': [90.0, -30.0, 60.0, -90.0, 0.0, 45.0],
            'maintenance': [180.0, -60.0, 120.0, -45.0, 90.0, 0.0],
            'custom': [30.0, -15.0, 30.0, -30.0, 45.0, 60.0]
        }
        
        # Joint names - update these to match your URDF exactly
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        
        # Create publisher for Isaac Sim joint commands
        self.joint_command_pub = self.create_publisher(
            JointState,
            '/isaac/joint_command',
            10
        )
        
        # Log startup info
        self.get_logger().info('Canadarm2 Preset GUI Node started')
        self.get_logger().info(f'Publishing to: /isaac/joint_command')
        self.get_logger().info(f'Available presets: {list(self.presets.keys())}')
        
        # Create GUI in main thread
        self.create_gui()
    
    def create_gui(self):
        """Create the tkinter GUI"""
        # Create main window
        self.window = tk.Tk()
        self.window.title("Canadarm2 Preset Controller")
        self.window.geometry("350x600")
        self.window.resizable(False, False)
        
        # Set window background
        self.window.configure(bg='#2c3e50')
        
        # Title
        title = tk.Label(
            self.window, 
            text="CANADARM2", 
            font=("Arial", 20, "bold"),
            bg='#2c3e50',
            fg='white'
        )
        title.pack(pady=20)
        
        subtitle = tk.Label(
            self.window, 
            text="ROS2 Preset Controller", 
            font=("Arial", 12),
            bg='#2c3e50',
            fg='#bdc3c7'
        )
        subtitle.pack(pady=(0, 20))
        
        # Preset buttons configuration
        button_configs = [
            ('HOME', 'home', '#27ae60'),
            ('STOW', 'stow', '#3498db'),
            ('DEPLOY', 'deploy', '#e74c3c'),
            ('INSPECT', 'inspect', '#f39c12'),
            ('MAINTENANCE', 'maintenance', '#9b59b6'),
            ('CUSTOM', 'custom', '#34495e')
        ]
        
        # Create preset buttons
        for display_name, preset_name, color in button_configs:
            # Get positions for display
            positions = self.presets.get(preset_name, [0.0] * 6)
            
            btn = tk.Button(
                self.window,
                text=f"{display_name}\n{positions}°",
                command=lambda p=preset_name: self.send_preset(p),
                width=25,
                height=3,
                font=("Arial", 10, "bold"),
                bg=color,
                fg="white",
                relief="raised",
                bd=3,
                activebackground=self.darken_color(color),
                cursor="hand2",
                wraplength=300
            )
            btn.pack(pady=8)
        
        # Status frame
        status_frame = tk.Frame(self.window, bg='#2c3e50')
        status_frame.pack(pady=20)
        
        tk.Label(
            status_frame, 
            text="Status:", 
            font=("Arial", 10, "bold"),
            bg='#2c3e50',
            fg='white'
        ).pack()
        
        self.status_label = tk.Label(
            status_frame, 
            text="Ready", 
            font=("Arial", 12),
            bg='#2c3e50',
            fg="#27ae60"
        )
        self.status_label.pack()
        
        # ROS info display
        ros_info = tk.Label(
            status_frame,
            text=f"Publishing to: /isaac/joint_command",
            font=("Arial", 8),
            bg='#2c3e50',
            fg='#95a5a6'
        )
        ros_info.pack(pady=(10, 0))
        
        # Control buttons frame
        control_frame = tk.Frame(self.window, bg='#2c3e50')
        control_frame.pack(pady=20)
        
        # Emergency stop button
        emergency_btn = tk.Button(
            control_frame,
            text="EMERGENCY STOP",
            command=self.emergency_stop,
            width=15,
            height=1,
            font=("Arial", 10, "bold"),
            bg="#c0392b",
            fg="white",
            relief="raised",
            bd=3
        )
        emergency_btn.pack(pady=5)
        
        # Quit button
        quit_btn = tk.Button(
            control_frame,
            text="Exit",
            command=self.quit_app,
            width=15,
            height=1,
            font=("Arial", 10),
            bg="#7f8c8d",
            fg="white",
            relief="raised",
            bd=2
        )
        quit_btn.pack(pady=5)
    
    def degrees_to_radians(self, degrees_list):
        """Convert degrees to radians"""
        return [math.radians(deg) for deg in degrees_list]
    
    def darken_color(self, color):
        """Darken a hex color for hover effect"""
        color_map = {
            '#27ae60': '#229954',
            '#3498db': '#2980b9', 
            '#e74c3c': '#c0392b',
            '#f39c12': '#e67e22',
            '#9b59b6': '#8e44ad',
            '#34495e': '#2c3e50'
        }
        return color_map.get(color, color)
    
    def send_preset(self, preset_name):
        """Send preset command to Isaac Sim via ROS2"""
        try:
            if preset_name not in self.presets:
                self.get_logger().error(f"Unknown preset: {preset_name}")
                self.status_label.config(text=f"✗ Unknown preset: {preset_name}", fg="#e74c3c")
                return
            
            # Update status immediately
            positions_deg = self.presets[preset_name]
            self.status_label.config(text=f"Sending {preset_name.upper()}...", fg="#f39c12")
            self.window.update()
            
            # Get positions in degrees and convert to radians
            positions_rad = self.degrees_to_radians(positions_deg)
            
            # Create JointState message
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.header.frame_id = ""
            joint_msg.name = self.joint_names[:len(positions_rad)]  # Use only needed joints
            joint_msg.position = positions_rad
            joint_msg.velocity = []  # Empty - Isaac Sim will handle
            joint_msg.effort = []    # Empty - Isaac Sim will handle
            
            # Publish the message
            self.joint_command_pub.publish(joint_msg)
            
            # Update status
            self.status_label.config(
                text=f"✓ {preset_name.upper()} sent: {positions_deg}°", 
                fg="#27ae60"
            )
            
            # Log the action
            self.get_logger().info(
                f"Sent preset '{preset_name}': {positions_deg}° -> {[round(r, 4) for r in positions_rad]} rad"
            )
            
            # Reset status after 3 seconds
            self.window.after(3000, lambda: self.status_label.config(text="Ready", fg="#27ae60"))
            
        except Exception as e:
            self.status_label.config(text="✗ Failed to send preset", fg="#e74c3c")
            self.get_logger().error(f"Error sending preset: {e}")
    
    def emergency_stop(self):
        """Send emergency stop command (home position)"""
        self.status_label.config(text="EMERGENCY STOP!", fg="#c0392b")
        self.get_logger().warn("Emergency stop activated - sending home position")
        self.send_preset('home')
    
    def quit_app(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down Canadarm2 Preset GUI")
        self.window.quit()
        self.window.destroy()
    
    def run_gui(self):
        """Run the GUI main loop"""
        # Center window on screen
        self.window.update_idletasks()
        x = (self.window.winfo_screenwidth() // 2) - (self.window.winfo_width() // 2)
        y = (self.window.winfo_screenheight() // 2) - (self.window.winfo_height() // 2)
        self.window.geometry(f"+{x}+{y}")
        
        # Set up clean shutdown
        self.window.protocol("WM_DELETE_WINDOW", self.quit_app)
        
        self.get_logger().info("GUI window opened")
        self.get_logger().info("Available presets:")
        for name, positions in self.presets.items():
            self.get_logger().info(f"  {name}: {positions}°")
        
        # Start the GUI main loop
        self.window.mainloop()

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create the node
        node = CanadarmPresetGUI()
        
        # Run ROS2 spinning in a separate thread
        def spin_ros():
            try:
                rclpy.spin(node)
            except Exception as e:
                node.get_logger().error(f"ROS spinning error: {e}")
        
        spin_thread = threading.Thread(target=spin_ros, daemon=True)
        spin_thread.start()
        
        # Run the GUI in the main thread
        node.run_gui()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean shutdown
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

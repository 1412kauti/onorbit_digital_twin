#!/usr/bin/env python3

import tkinter as tk
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import threading
import math
import sys

class CanadarmMoveItPresetGUI(Node):
    def __init__(self):
        super().__init__('canadarm_moveit_preset_gui')
        
        # Define your preset positions in degrees
        self.presets = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'stow': [0.0, -90.0, 90.0, 0.0, -90.0, 0.0],
            'deploy': [45.0, -45.0, 45.0, 0.0, -45.0, 90.0],
            'inspect': [90.0, -30.0, 60.0, -90.0, 0.0, 45.0],
            'maintenance': [180.0, -60.0, 120.0, -45.0, 90.0, 0.0],
            'custom': [30.0, -15.0, 30.0, -30.0, 45.0, 60.0]
        }
        
        # Joint names - update these to match your MoveIt config
        self.joint_names = [
            "joint_1", "joint_2", "joint_3", 
            "joint_4", "joint_5", "joint_6"
        ]
        
        # Create publisher for MoveIt joint trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        # Execution time for trajectories (seconds)
        self.execution_time = 5.0
        
        # Log startup info
        self.get_logger().info('Canadarm2 MoveIt Preset GUI Node started')
        self.get_logger().info(f'Publishing to: /arm_controller/joint_trajectory')
        self.get_logger().info(f'Available presets: {list(self.presets.keys())}')
        
        # Create GUI in main thread
        self.create_gui()
    
    def create_gui(self):
        """Create the tkinter GUI"""
        # Create main window
        self.window = tk.Tk()
        self.window.title("Canadarm2 MoveIt Controller")
        self.window.geometry("350x650")
        self.window.resizable(False, False)
        
        # Set window background
        self.window.configure(bg='#34495e')
        
        # Title
        title = tk.Label(
            self.window, 
            text="CANADARM2", 
            font=("Arial", 20, "bold"),
            bg='#34495e',
            fg='white'
        )
        title.pack(pady=20)
        
        subtitle = tk.Label(
            self.window, 
            text="MoveIt Trajectory Controller", 
            font=("Arial", 12),
            bg='#34495e',
            fg='#ecf0f1'
        )
        subtitle.pack(pady=(0, 20))
        
        # Execution time control
        time_frame = tk.Frame(self.window, bg='#34495e')
        time_frame.pack(pady=10)
        
        tk.Label(
            time_frame,
            text="Execution Time:",
            font=("Arial", 10),
            bg='#34495e',
            fg='white'
        ).pack(side=tk.LEFT)
        
        self.time_var = tk.DoubleVar(value=self.execution_time)
        time_spinbox = tk.Spinbox(
            time_frame,
            from_=1.0,
            to=10.0,
            increment=0.5,
            textvariable=self.time_var,
            width=5,
            command=self.update_execution_time
        )
        time_spinbox.pack(side=tk.LEFT, padx=(5, 0))
        
        tk.Label(
            time_frame,
            text="seconds",
            font=("Arial", 10),
            bg='#34495e',
            fg='white'
        ).pack(side=tk.LEFT, padx=(5, 0))
        
        # Preset buttons configuration
        button_configs = [
            ('HOME', 'home', '#2ecc71'),
            ('STOW', 'stow', '#3498db'),
            ('DEPLOY', 'deploy', '#e74c3c'),
            ('INSPECT', 'inspect', '#f39c12'),
            ('MAINTENANCE', 'maintenance', '#9b59b6'),
            ('CUSTOM', 'custom', '#95a5a6')
        ]
        
        # Create preset buttons
        for display_name, preset_name, color in button_configs:
            # Get positions for display
            positions = self.presets.get(preset_name, [0.0] * 6)
            
            btn = tk.Button(
                self.window,
                text=f"{display_name}\n{positions}°",
                command=lambda p=preset_name: self.send_trajectory(p),
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
        status_frame = tk.Frame(self.window, bg='#34495e')
        status_frame.pack(pady=20)
        
        tk.Label(
            status_frame, 
            text="Status:", 
            font=("Arial", 10, "bold"),
            bg='#34495e',
            fg='white'
        ).pack()
        
        self.status_label = tk.Label(
            status_frame, 
            text="Ready", 
            font=("Arial", 12),
            bg='#34495e',
            fg="#2ecc71"
        )
        self.status_label.pack()
        
        # MoveIt info display
        moveit_info = tk.Label(
            status_frame,
            text=f"Publishing to: /arm_controller/joint_trajectory",
            font=("Arial", 8),
            bg='#34495e',
            fg='#bdc3c7'
        )
        moveit_info.pack(pady=(10, 0))
        
        # Control buttons frame
        control_frame = tk.Frame(self.window, bg='#34495e')
        control_frame.pack(pady=20)
        
        # Stop trajectory button
        stop_btn = tk.Button(
            control_frame,
            text="STOP TRAJECTORY",
            command=self.stop_trajectory,
            width=15,
            height=1,
            font=("Arial", 10, "bold"),
            bg="#e67e22",
            fg="white",
            relief="raised",
            bd=3
        )
        stop_btn.pack(pady=5)
        
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
    
    def update_execution_time(self):
        """Update execution time from spinbox"""
        self.execution_time = self.time_var.get()
        self.get_logger().info(f"Execution time updated to: {self.execution_time}s")
    
    def degrees_to_radians(self, degrees_list):
        """Convert degrees to radians"""
        return [math.radians(deg) for deg in degrees_list]
    
    def darken_color(self, color):
        """Darken a hex color for hover effect"""
        color_map = {
            '#2ecc71': '#27ae60',
            '#3498db': '#2980b9', 
            '#e74c3c': '#c0392b',
            '#f39c12': '#e67e22',
            '#9b59b6': '#8e44ad',
            '#95a5a6': '#7f8c8d'
        }
        return color_map.get(color, color)
    
    def send_trajectory(self, preset_name):
        """Send trajectory command to MoveIt via ROS2"""
        try:
            if preset_name not in self.presets:
                self.get_logger().error(f"Unknown preset: {preset_name}")
                self.status_label.config(text=f"✗ Unknown preset: {preset_name}", fg="#e74c3c")
                return
            
            # Update status immediately
            positions_deg = self.presets[preset_name]
            self.status_label.config(text=f"Executing {preset_name.upper()}...", fg="#f39c12")
            self.window.update()
            
            # Get positions in degrees and convert to radians
            positions_rad = self.degrees_to_radians(positions_deg)
            
            # Create JointTrajectory message
            trajectory_msg = JointTrajectory()
            trajectory_msg.header.stamp = self.get_clock().now().to_msg()
            trajectory_msg.header.frame_id = ""
            trajectory_msg.joint_names = self.joint_names[:len(positions_rad)]
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = positions_rad
            point.velocities = [0.0] * len(positions_rad)  # Stop at target
            point.accelerations = [0.0] * len(positions_rad)  # Smooth motion
            
            # Set execution time
            point.time_from_start = Duration(sec=int(self.execution_time), 
                                           nanosec=int((self.execution_time % 1) * 1e9))
            
            trajectory_msg.points = [point]
            
            # Publish the trajectory
            self.trajectory_pub.publish(trajectory_msg)
            
            # Update status
            self.status_label.config(
                text=f"✓ {preset_name.upper()} trajectory sent: {positions_deg}°", 
                fg="#2ecc71"
            )
            
            # Log the action
            self.get_logger().info(
                f"Sent trajectory '{preset_name}': {positions_deg}° -> {[round(r, 4) for r in positions_rad]} rad in {self.execution_time}s"
            )
            
            # Reset status after execution time + 1 second
            reset_time = int((self.execution_time + 1) * 1000)
            self.window.after(reset_time, lambda: self.status_label.config(text="Ready", fg="#2ecc71"))
            
        except Exception as e:
            self.status_label.config(text="✗ Failed to send trajectory", fg="#e74c3c")
            self.get_logger().error(f"Error sending trajectory: {e}")
    
    def stop_trajectory(self):
        """Send empty trajectory to stop current motion"""
        try:
            # Create empty trajectory to stop motion
            trajectory_msg = JointTrajectory()
            trajectory_msg.header.stamp = self.get_clock().now().to_msg()
            trajectory_msg.joint_names = self.joint_names
            trajectory_msg.points = []  # Empty points array stops trajectory
            
            self.trajectory_pub.publish(trajectory_msg)
            
            self.status_label.config(text="Trajectory stopped", fg="#e67e22")
            self.get_logger().warn("Trajectory execution stopped")
            
            self.window.after(2000, lambda: self.status_label.config(text="Ready", fg="#2ecc71"))
            
        except Exception as e:
            self.get_logger().error(f"Error stopping trajectory: {e}")
    
    def emergency_stop(self):
        """Send emergency stop command (immediate home position)"""
        self.status_label.config(text="EMERGENCY STOP!", fg="#c0392b")
        self.get_logger().warn("Emergency stop activated - sending immediate home position")
        
        # Save current execution time and set to fast
        original_time = self.execution_time
        self.execution_time = 1.0
        
        # Send home position quickly
        self.send_trajectory('home')
        
        # Restore original execution time
        self.execution_time = original_time
        self.time_var.set(self.execution_time)
    
    def quit_app(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down Canadarm2 MoveIt Preset GUI")
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
        
        self.get_logger().info("MoveIt GUI window opened")
        self.get_logger().info("Available presets:")
        for name, positions in self.presets.items():
            self.get_logger().info(f"  {name}: {positions}°")
        self.get_logger().info(f"Default execution time: {self.execution_time}s")
        
        # Start the GUI main loop
        self.window.mainloop()

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create the node
        node = CanadarmMoveItPresetGUI()
        
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

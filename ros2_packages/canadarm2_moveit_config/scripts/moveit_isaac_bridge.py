#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
import threading
import time

class MoveItIsaacBridge(Node):
    def __init__(self):
        super().__init__('moveit_isaac_bridge')
        
        # Isaac Sim joint order (from /isaac/joint_states)
        self.isaac_joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        
        # Current joint positions
        self.current_positions = [0.0] * 7
        self.position_lock = threading.Lock()
        
        # Subscribe to Isaac Sim joint states (feedback)
        self.isaac_state_sub = self.create_subscription(
            JointState,
            '/isaac/joint_states',
            self.isaac_state_callback,
            10
        )
        
        # Subscribe to MoveIt trajectory commands
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        # Publisher to Isaac Sim
        self.isaac_command_pub = self.create_publisher(
            JointState,
            '/isaac/joint_command',
            10
        )
        
        # Publisher for MoveIt feedback
        self.controller_state_pub = self.create_publisher(
            JointTrajectoryControllerState,
            '/arm_controller/state',
            10
        )
        
        # Action server for FollowJointTrajectory
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.execute_trajectory_action
        )
        
        # Timer for publishing controller state
        self.state_timer = self.create_timer(0.1, self.publish_controller_state)
        
        self.get_logger().info('Enhanced MoveIt-Isaac bridge started')
        self.get_logger().info('Services:')
        self.get_logger().info('  - Isaac feedback: /isaac/joint_states -> /arm_controller/state')
        self.get_logger().info('  - MoveIt commands: /arm_controller/joint_trajectory -> /isaac/joint_command')
        self.get_logger().info('  - Action server: /arm_controller/follow_joint_trajectory')
    
    def isaac_state_callback(self, msg):
        """Update current joint positions from Isaac Sim"""
        with self.position_lock:
            if len(msg.position) >= 7:
                self.current_positions = list(msg.position[:7])
    
    def publish_controller_state(self):
        """Publish controller state for MoveIt feedback"""
        with self.position_lock:
            current_pos = self.current_positions.copy()
        
        # Create controller state message
        state_msg = JointTrajectoryControllerState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.joint_names = self.isaac_joint_names
        
        # Actual state (from Isaac Sim)
        state_msg.actual.positions = current_pos
        state_msg.actual.velocities = [0.0] * 7
        state_msg.actual.accelerations = [0.0] * 7
        
        # Desired state (same as actual for now)
        state_msg.desired.positions = current_pos
        state_msg.desired.velocities = [0.0] * 7
        state_msg.desired.accelerations = [0.0] * 7
        
        # Error (zero for now)
        state_msg.error.positions = [0.0] * 7
        state_msg.error.velocities = [0.0] * 7
        state_msg.error.accelerations = [0.0] * 7
        
        self.controller_state_pub.publish(state_msg)
    
    def trajectory_callback(self, msg):
        """Convert trajectory to joint state for Isaac Sim"""
        if not msg.points:
            self.get_logger().warn('Received empty trajectory')
            return
        
        self.get_logger().info(f'Received trajectory with {len(msg.points)} points')
        
        # Send each point in the trajectory to Isaac Sim
        for i, point in enumerate(msg.points):
            # Create JointState message for Isaac Sim
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = self.isaac_joint_names
            joint_msg.position = list(point.positions)
            
            # Publish to Isaac Sim
            self.isaac_command_pub.publish(joint_msg)
            
            if i == 0:  # Log first point
                self.get_logger().info(f'Sending trajectory point to Isaac Sim: {[round(p, 3) for p in point.positions]}')
            
            # Small delay between points for smooth motion
            if i < len(msg.points) - 1:
                time.sleep(0.05)
    
    def execute_trajectory_action(self, goal_handle):
        """Execute FollowJointTrajectory action"""
        self.get_logger().info('Executing FollowJointTrajectory action')
        
        trajectory = goal_handle.request.trajectory
        
        # Forward to Isaac Sim via trajectory callback
        self.trajectory_callback(trajectory)
        
        # Wait for trajectory completion (simplified)
        if trajectory.points:
            total_time = 0.0
            if trajectory.points[-1].time_from_start.sec > 0 or trajectory.points[-1].time_from_start.nanosec > 0:
                total_time = trajectory.points[-1].time_from_start.sec + trajectory.points[-1].time_from_start.nanosec * 1e-9
            else:
                total_time = len(trajectory.points) * 0.1  # Default timing
            
            self.get_logger().info(f'Waiting {total_time:.2f}s for trajectory completion')
            time.sleep(total_time)
        
        # Return success
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result

def main():
    rclpy.init()
    bridge = MoveItIsaacBridge()
    
    # Use MultiThreadedExecutor for action server
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

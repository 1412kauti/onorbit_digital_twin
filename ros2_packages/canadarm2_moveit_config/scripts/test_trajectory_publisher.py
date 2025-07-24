#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time

class TestTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('test_trajectory_publisher')
        
        # Publisher for joint trajectory
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        # Joint names matching Isaac Sim
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        
        # Predefined poses
        self.poses = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'pose1': [0.5, -0.3, 0.2, -0.5, 0.3, 0.2, -0.1],
            'pose2': [-0.3, 0.5, -0.2, 0.3, -0.5, 0.4, 0.2],
            'pose3': [0.2, -0.6, 0.4, -0.2, 0.6, -0.3, -0.4]
        }
        
        self.get_logger().info('Test Trajectory Publisher started')
        self.get_logger().info('Available poses: ' + ', '.join(self.poses.keys()))
        self.get_logger().info('Use: ros2 service call /test_trajectory_publisher/send_pose std_srvs/srv/SetBool "{data: true}"')
        
        # Timer to send test trajectories
        self.create_timer(5.0, self.send_test_trajectory)
        self.pose_index = 0

    def create_trajectory_msg(self, target_positions, duration_sec=2.0):
        """Create a JointTrajectory message"""
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        trajectory.points = [point]
        return trajectory

    def send_test_trajectory(self):
        """Send a test trajectory periodically"""
        pose_names = list(self.poses.keys())
        current_pose_name = pose_names[self.pose_index % len(pose_names)]
        target_positions = self.poses[current_pose_name]
        
        # Create and publish trajectory
        trajectory = self.create_trajectory_msg(target_positions, 3.0)
        self.trajectory_pub.publish(trajectory)
        
        self.get_logger().info(f'Published trajectory to {current_pose_name}: {[round(p, 3) for p in target_positions]}')
        
        self.pose_index += 1

def main():
    rclpy.init()
    
    try:
        publisher = TestTrajectoryPublisher()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

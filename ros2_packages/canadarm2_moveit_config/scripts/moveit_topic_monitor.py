#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import (
    PlanningScene, 
    DisplayTrajectory, 
    AttachedCollisionObject,
    CollisionObject,
    MotionPlanRequest
)
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.msg import ControllerState  
from lifecycle_msgs.msg import TransitionEvent
import json
from datetime import datetime

class MoveItTopicMonitor(Node):
    def __init__(self):
        super().__init__('moveit_topic_monitor')
        
        # QoS profiles
        self.default_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Topic counters and storage
        self.topic_counts = {}
        self.last_messages = {}
        
        # Create subscribers
        self.create_subscribers()
        
        # Timer for status updates
        self.create_timer(10.0, self.print_status)
        
        self.get_logger().info("MoveIt Topic Monitor started for Canadarm2!")
        self.get_logger().info("Monitoring topics: " + ", ".join(self.get_monitored_topics()))

    def get_monitored_topics(self):
        return [
            '/arm_controller/controller_state',
            '/arm_controller/joint_trajectory', 
            '/arm_controller/state',
            '/arm_controller/transition_event',
            '/attached_collision_object',
            '/collision_object',
            '/display_planned_path',
            '/dynamic_joint_states',
            '/joint_state_broadcaster/transition_event',
            '/joint_states',
            '/monitored_planning_scene',
            '/motion_plan_request',
            '/isaac/joint_states',
            '/isaac/joint_command'
        ]

    def create_subscribers(self):
        # Controller topics
        self.create_subscription(ControllerState, '/arm_controller/controller_state',
                               lambda msg: self.callback('/arm_controller/controller_state', msg), self.default_qos)
        
        self.create_subscription(JointTrajectory, '/arm_controller/joint_trajectory',
                               lambda msg: self.callback('/arm_controller/joint_trajectory', msg), self.default_qos)
        
        self.create_subscription(JointTrajectoryControllerState, '/arm_controller/state',
                               lambda msg: self.callback('/arm_controller/state', msg), self.default_qos)
        
        self.create_subscription(TransitionEvent, '/arm_controller/transition_event',
                               lambda msg: self.callback('/arm_controller/transition_event', msg), self.reliable_qos)
        
        # MoveIt topics
        self.create_subscription(AttachedCollisionObject, '/attached_collision_object',
                               lambda msg: self.callback('/attached_collision_object', msg), self.reliable_qos)
        
        self.create_subscription(CollisionObject, '/collision_object',
                               lambda msg: self.callback('/collision_object', msg), self.reliable_qos)
        
        self.create_subscription(DisplayTrajectory, '/display_planned_path',
                               lambda msg: self.callback('/display_planned_path', msg), self.default_qos)
        
        # Joint state topics
        self.create_subscription(JointState, '/dynamic_joint_states',
                               lambda msg: self.callback('/dynamic_joint_states', msg), self.default_qos)
        
        self.create_subscription(TransitionEvent, '/joint_state_broadcaster/transition_event',
                               lambda msg: self.callback('/joint_state_broadcaster/transition_event', msg), self.reliable_qos)
        
        self.create_subscription(JointState, '/joint_states',
                               lambda msg: self.callback('/joint_states', msg), self.default_qos)
        
        # Planning topics
        self.create_subscription(PlanningScene, '/monitored_planning_scene',
                               lambda msg: self.callback('/monitored_planning_scene', msg), self.reliable_qos)
        
        self.create_subscription(MotionPlanRequest, '/motion_plan_request',
                               lambda msg: self.callback('/motion_plan_request', msg), self.reliable_qos)
        
        # Isaac Sim topics
        self.create_subscription(JointState, '/isaac/joint_states',
                               lambda msg: self.callback('/isaac/joint_states', msg), self.default_qos)
        
        self.create_subscription(JointState, '/isaac/joint_command',
                               lambda msg: self.callback('/isaac/joint_command', msg), self.default_qos)

    def callback(self, topic_name, msg):
        # Update counter
        self.topic_counts[topic_name] = self.topic_counts.get(topic_name, 0) + 1
        
        # Extract key message info
        msg_info = {
            'timestamp': datetime.now().strftime('%H:%M:%S.%f')[:-3],
            'type': type(msg).__name__,
            'count': self.topic_counts[topic_name]
        }
        
        # Add message-specific details
        if isinstance(msg, JointState):
            msg_info['joints'] = len(msg.name)
            if msg.position:
                msg_info['positions'] = [round(p, 3) for p in msg.position[:3]]  # First 3 only
        
        elif isinstance(msg, JointTrajectory):
            msg_info['joints'] = len(msg.joint_names)
            msg_info['points'] = len(msg.points)
        
        elif isinstance(msg, JointTrajectoryControllerState):
            msg_info['joints'] = len(msg.joint_names)
            if msg.actual.positions:
                msg_info['actual_pos'] = [round(p, 3) for p in msg.actual.positions[:3]]
        
        elif isinstance(msg, CollisionObject):
            msg_info['id'] = msg.id
            msg_info['operation'] = msg.operation
        
        elif isinstance(msg, AttachedCollisionObject):
            msg_info['link'] = msg.link_name
            msg_info['object_id'] = msg.object.id if msg.object else 'none'
        
        elif isinstance(msg, DisplayTrajectory):
            msg_info['model_id'] = msg.model_id
            msg_info['trajectories'] = len(msg.trajectory)
        
        elif isinstance(msg, MotionPlanRequest):
            msg_info['group'] = msg.group_name
            msg_info['goals'] = len(msg.goal_constraints)
        
        elif isinstance(msg, PlanningScene):
            msg_info['name'] = msg.name
            msg_info['diff'] = msg.is_diff
        
        elif isinstance(msg, ControllerState):
            msg_info['name'] = msg.name
            msg_info['state'] = msg.state
        
        elif isinstance(msg, TransitionEvent):
            msg_info['transition'] = f"{msg.start_state.label} -> {msg.goal_state.label}"
        
        self.last_messages[topic_name] = msg_info
        
        # Log concisely
        details = ', '.join([f"{k}:{v}" for k, v in msg_info.items() if k not in ['timestamp', 'type', 'count']])
        self.get_logger().info(f"[{topic_name.split('/')[-1]:20}] #{msg_info['count']:3} | {details}")

    def print_status(self):
        if not self.topic_counts:
            self.get_logger().info("No MoveIt messages received yet...")
            return
        
        self.get_logger().info(f"\n{'='*70}")
        self.get_logger().info("CANADARM2 MOVEIT TOPIC MONITOR STATUS")
        self.get_logger().info(f"{'='*70}")
        
        # Sort by activity
        sorted_topics = sorted(self.topic_counts.items(), key=lambda x: x[1], reverse=True)
        
        for topic, count in sorted_topics:
            last_msg = self.last_messages.get(topic, {})
            last_time = last_msg.get('timestamp', 'Never')
            msg_type = last_msg.get('type', 'Unknown')
            
            self.get_logger().info(f"{topic:35} | {count:4} msgs | {msg_type:20} | {last_time}")
        
        self.get_logger().info(f"{'='*70}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = MoveItTopicMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if 'monitor' in locals():
            monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

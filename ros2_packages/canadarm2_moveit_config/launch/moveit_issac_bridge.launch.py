from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Add to your existing MoveIt launch file
	Node(
    		package='canadarm2_moveit_config',
    		executable='moveit_isaac_bridge.py',
    		name='moveit_isaac_bridge',
    		output='screen'
		),
    ])

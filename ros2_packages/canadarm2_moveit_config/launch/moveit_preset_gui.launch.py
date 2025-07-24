from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='canadarm2_moveit_config',
            executable='moveit_preset_gui.py',
            name='canadarm_moveit_preset_gui',
            output='screen'
        )
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'canadarm2_description'
    urdf_file = 'CanadaArm2-PLY-ROS2.urdf'
    rviz_config_file = 'canadarm2.rviz'

    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share,'urdf',urdf_file)
    rviz_path = os.path.join(pkg_share,'rviz',rviz_config_file)

    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        # Static transform publisher: map -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_map_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        # RViz with RobotModel display
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            output='screen'
        )
    ])
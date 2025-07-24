from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directories
    pkg_canadarm2_moveit_config = FindPackageShare('canadarm2_moveit_config')
    
    # Launch arguments  
    launch_move_group_arg = DeclareLaunchArgument(
        'launch_move_group',
        default_value='true',
        description='Whether to launch MoveIt move_group node'
    )
    
    # Include MoveIt move_group launch (optional)
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_canadarm2_moveit_config,
                'launch',
                'move_group.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('launch_move_group'))
    )
    
    # MoveIt Topic Monitor Node
    moveit_monitor_node = Node(
        package='canadarm2_moveit_config',
        executable='moveit_topic_monitor.py',
        name='moveit_topic_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        emulate_tty=True
    )
    
    return LaunchDescription([
        launch_move_group_arg,
        move_group_launch,
        moveit_monitor_node
    ])

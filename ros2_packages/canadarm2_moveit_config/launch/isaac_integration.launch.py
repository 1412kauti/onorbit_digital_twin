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
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Whether to launch RViz'
    )
    
    enable_test_publisher_arg = DeclareLaunchArgument(
        'enable_test_publisher',
        default_value='false',
        description='Whether to launch test trajectory publisher'
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
    
    # Include RViz launch (optional)
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_canadarm2_moveit_config,
                'launch',
                'moveit_rviz.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )
    
    # MoveIt-Isaac Bridge (the key component)
    isaac_bridge_node = Node(
        package='canadarm2_moveit_config',
        executable='moveit_isaac_bridge.py',
        name='moveit_isaac_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        emulate_tty=True
    )
    
    # Topic Monitor
    topic_monitor_node = Node(
        package='canadarm2_moveit_config',
        executable='moveit_topic_monitor.py',
        name='moveit_topic_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        emulate_tty=True
    )
    
    # Test Trajectory Publisher (optional)
    test_publisher_node = Node(
        package='canadarm2_moveit_config',
        executable='test_trajectory_publisher.py',
        name='test_trajectory_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_test_publisher')),
        parameters=[{
            'use_sim_time': False,
        }],
        emulate_tty=True
    )
    
    return LaunchDescription([
        launch_move_group_arg,
        launch_rviz_arg,
        enable_test_publisher_arg,
        move_group_launch,
        rviz_launch,
        isaac_bridge_node,
        topic_monitor_node,
        test_publisher_node
    ])

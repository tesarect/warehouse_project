from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Declare launch arguments
    obstacle_arg = DeclareLaunchArgument(
        'obstacle',
        default_value='0.3',
        description='Distance to obstacle in meters'
    )
    
    degrees_arg = DeclareLaunchArgument(
        'degrees',
        default_value='-90',
        description='Degrees to rotate after stopping'
    )
    
    final_approach_arg = DeclareLaunchArgument(
        'final_approach',
        default_value='false',
        description='Whether to perform final approach and attach to shelf'
    )

    # Get launch argument values
    obstacle = LaunchConfiguration('obstacle')
    degrees = LaunchConfiguration('degrees')
    final_approach = LaunchConfiguration('final_approach')

    # Find package share directory for rviz config
    pkg_share = FindPackageShare('attach_shelf').find('attach_shelf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'warehouse_rb1.rviz')

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Approach service server node
    approach_service_server_node = Node(
        package='attach_shelf',
        executable='approach_service_server',
        name='approach_service_server',
        output='screen'
    )

    # Pre-approach V2 node (starts after 5 second delay)
    pre_approach_v2_node = Node(
        package='attach_shelf',
        executable='pre_approach_v2',
        name='pre_approach_v2_node',
        output='screen',
        parameters=[{
            'obstacle': obstacle,
            'degrees': degrees,
            'final_approach': final_approach
        }],
        prefix='bash -c \'sleep 3; $0 $@\''
    )

    return LaunchDescription([
        # Declare arguments
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        
        # Launch nodes
        rviz_node,
        approach_service_server_node,
        pre_approach_v2_node
    ])
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    map_file = LaunchConfiguration('map_file').perform(context)
    
    # Determine if simulation based on map file name
    is_sim = 'warehouse_map_sim.yaml' in map_file
    use_sim_time_value = True if is_sim else False
    
    print(f"Map file: {map_file}")
    print(f"Is simulation: {is_sim}")
    print(f"use_sim_time: {use_sim_time_value}")
    
    rviz_config = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'pathplanning.rviz')
    map_dir = os.path.join(get_package_share_directory('map_server'), 'config')
    
    # Select the appropriate AMCL config file
    loc_config_dir = os.path.join(get_package_share_directory('localization_server'), 'config')
    amcl_config_file = os.path.join(loc_config_dir, f'amcl_config_{"sim" if is_sim else "real"}.yaml')
    
    print(f"AMCL config file: {amcl_config_file}")
    
    # Make sure map file path is constructed correctly
    map_file_path = os.path.join(map_dir, map_file)
    print(f"Full map path: {map_file_path}")
    
    nodes = [
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': use_sim_time_value}],
            arguments=['-d', rviz_config]),
            
        # Make sure to pass the full path to the map file
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time_value},
                {'yaml_filename': map_file_path}
            ]),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                amcl_config_file,
                {'use_sim_time': use_sim_time_value}
            ]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time_value},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]),

        # Updated static transform publisher with new-style arguments
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_footprint_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0', 
                      '--roll', '0', '--pitch', '0', '--yaw', '0', 
                      '--frame-id', 'robot_base_footprint', 
                      '--child-frame-id', 'robot_base_link']),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('localization_server'),
                    'launch',
                    'init_robot.launch.py'
                ])
            )
        ),
    ]
    
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value='warehouse_map_sim.yaml',
            description='Name of the map file to use'),
            
        OpaqueFunction(function=launch_setup)
    ])

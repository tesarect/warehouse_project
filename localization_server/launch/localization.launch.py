import os
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    rviz_config = os.path.join(get_package_share_directory('map_server'), 'rviz', '3rd_prsn_view.rviz')
    # rviz_config = os.path.join(get_package_share_directory('map_server'), 'rviz', 'map_display.rviz')
    nav2_amcl = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_sim.yaml')

    map_file = LaunchConfiguration('map_file', default='warehouse_map_sim.yaml')
    map_dir = os.path.join(get_package_share_directory('map_server'), 'config')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Set up conditional parameters based on map_file
    is_sim = PythonExpression(['"warehouse_map_sim.yaml" in "', map_file, '"'])

    return LaunchDescription([

        DeclareLaunchArgument(
            'map_file',
            default_value='warehouse_map_sim.yaml',
            description='Name of the map file to use'),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'),
        
        # Set use_sim_time based on map file
        SetEnvironmentVariable(
            name='USE_SIM_TIME',
            value=PythonExpression(['"true" if ', is_sim, ' else "false"'])),
        
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config]),   
            
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename':[map_dir, '/', map_file]}]),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_amcl]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]),

        # adding missing TF frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_footprint_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'robot_base_footprint', 'robot_base_link']),

        # Initial Position initializer  
        # Node(
        #     package='localization_server',
        #     executable='initial_pose_pub',
        #     output='screen'),
        # Initial Position initializer's  Launch file [need to try this]
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('localization_server'),
                    'launch',
                    'init_robot.launch.py'
                ])
            )
        ),
    ])
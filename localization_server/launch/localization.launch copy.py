import os  # For file and directory operations
from ament_index_python.packages import get_package_share_directory  # To get package share directories
from launch import LaunchDescription  # To create a launch description
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # To declare launch arguments
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration  # For substituting values
from launch_ros.actions import Node  # To define ROS 2 nodes
from launch_ros.substitutions import FindPackageShare  # To find package share directories
from launch.conditions import IfCondition, UnlessCondition  # For conditional node launching

def launch_setup(context, *args, **kwargs):
    
    # Get the map_file argument value
    map_file_name = LaunchConfiguration('map_file').perform(context)
    
    # Derive use_sim_time from map_file
    if 'sim' in map_file_name.lower():  # warehouse_map_keepout_sim.yaml
        use_sim_time = True
    elif 'real' in map_file_name.lower():  # warehouse_map_keepout_real.yaml
        use_sim_time = False
    else:       # TODO : need to remove this later
        # Default to sim if neither "sim" nor "real" in filename
        use_sim_time = True
        print(f"WARNING: Could not determine sim/real from map_file '{map_file_name}'. Defaulting to sim mode.")

    # Build the path to the map YAML file
    map_yaml = PathJoinSubstitution([
        FindPackageShare('map_server'), 'config', map_file_name
    ]).perform(context)

    pkg_path = get_package_share_directory('localization_server')

    # Define paths to AMCL configuration files for simulation and real-time
    amcl_yaml_sim = os.path.join(pkg_path, 'config', 'amcl_config_sim.yaml')
    amcl_yaml_real = os.path.join(pkg_path, 'config', 'amcl_config_real.yaml')

    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'localization.rviz')

    print(f'Localization Launch:')
    print(f'\t- map_file: {map_file_name}')
    print(f'\t- use_sim_time: {use_sim_time} (derived from map_file)')
    print(f'\t- map_yaml: {map_yaml}')

    # Define RViz node
    rviz_node = Node(
        package='rviz2', 
        executable='rviz2',
        name='rviz_node', 
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )

    # Define Map Server node
    map_server_node = Node(
        package='nav2_map_server', 
        executable='map_server',
        name='map_server', 
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml
        }]
    )

    # Define AMCL node for simulation
    amcl_node_sim = Node(
        package='nav2_amcl', 
        executable='amcl',
        name='amcl', 
        output='screen',
        parameters=[amcl_yaml_sim],
        condition=IfCondition(str(use_sim_time).lower())
    )

    # Define AMCL node for real-time
    amcl_node_real = Node(
        package='nav2_amcl', 
        executable='amcl',
        name='amcl', 
        output='screen',
        parameters=[amcl_yaml_real],
        condition=UnlessCondition(str(use_sim_time).lower())
    )

    # # Define AMCL node for simulation
    # amcl_node_sim = Node(
    #     package='nav2_amcl', 
    #     executable='amcl',
    #     name='amcl', 
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'yaml_filename': amcl_yaml_sim
    #     }],
    #     condition=IfCondition(str(use_sim_time).lower())
    # )

    # # Define AMCL node for real-time
    # amcl_node_real = Node(
    #     package='nav2_amcl', 
    #     executable='amcl',
    #     name='amcl', 
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'yaml_filename': amcl_yaml_real
    #     }],
    #     condition=UnlessCondition(str(use_sim_time).lower())
    # )

    # Define Lifecycle Manager node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager', 
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper', 
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # Return all nodes
    return [
        rviz_node,
        map_server_node,
        amcl_node_sim,
        amcl_node_real,
        lifecycle_manager_node,
    ]

def generate_launch_description():
    
    # Declare 'map_file' argument - ONLY argument for this launch file
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file', 
        default_value='warehouse_map_keepout_sim.yaml',
        description='Map file to load (use_sim_time derived automatically: *_sim.yaml -> True, *_real.yaml -> False)'
    )

    # Return the LaunchDescription with the argument and the opaque function
    return LaunchDescription([
        declare_map_file_cmd,
        OpaqueFunction(function=launch_setup)
    ])
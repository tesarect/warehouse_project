import os  # For file and directory operations
from ament_index_python.packages import get_package_share_directory  # To get package share directories
from launch import LaunchDescription  # To create a launch description
from launch.actions import DeclareLaunchArgument  # To declare launch arguments
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration  # For substituting values
from launch_ros.actions import Node  # To define ROS 2 nodes
from launch_ros.substitutions import FindPackageShare  # To find package share directories
from launch.conditions import IfCondition, UnlessCondition  # For conditional node launching

def generate_launch_description():
    
    # Configuration for using simulation time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare 'use_sim_time' argument with a default value
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time',
                                                     default_value='True',
                                                     description='Use simulation clock if True')

    # Declare 'map_file' argument with a default value
    map_file_arg = DeclareLaunchArgument(
        "map_file", default_value="warehouse_map_sim.yaml"
    )

    # Build the path to the map YAML file
    map_yaml = PathJoinSubstitution([
        FindPackageShare('map_server'), 'config', LaunchConfiguration("map_file")
    ])

    pkg_path = get_package_share_directory('localization_server')

    # Define paths to AMCL configuration files for simulation and real-time
    amcl_yaml_sim = os.path.join(pkg_path, 'config', 'amcl_config_sim.yaml')
    amcl_yaml_real = os.path.join(pkg_path, 'config', 'amcl_config_real.yaml')

    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'localization.rviz')

    # Define RViz node
    rviz_node = Node(package='rviz2', executable='rviz2',
                     name='rviz_node', output='screen',
                     parameters=[{'use_sim_time': use_sim_time}],
                     arguments=['-d', rviz_config_file])

    # Define Map Server node
    map_server_node = Node(package='nav2_map_server', executable='map_server',
                           name='map_server', output='screen',
                           parameters=[{'use_sim_time': use_sim_time},
                                       {'yaml_filename': map_yaml}])

    # Define AMCL node for simulation, conditional on using simulation time
    amcl_node_sim = Node(package='nav2_amcl', executable='amcl',
                         name='amcl', output='screen',
                         parameters=[amcl_yaml_sim],
                         condition=IfCondition(use_sim_time))

    # Define AMCL node for real-time, conditional on not using simulation time
    amcl_node_real = Node(package='nav2_amcl', executable='amcl',
                          name='amcl', output='screen',
                          parameters=[amcl_yaml_real],
                          condition=UnlessCondition(use_sim_time))

    # Define Lifecycle Manager node
    lifecycle_manager_node = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                                  name='lifecycle_manager_mapper', output='screen',
                                  parameters=[{'use_sim_time': use_sim_time},
                                              {'autostart': True},
                                              {'node_names': ['map_server', 'amcl']}]
                                 )

    # Return the LaunchDescription with all nodes and arguments
    return LaunchDescription([
        declare_use_sim_time_cmd,
        map_file_arg,
        rviz_node,
        map_server_node,
        amcl_node_sim,
        amcl_node_real,
        lifecycle_manager_node,
    ])

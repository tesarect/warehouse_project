from ament_index_python.packages import get_package_share_directory 
import os  # For file and directory path operations
from ament_index_python.packages import get_package_share_directory  # Get share directory of a package
from launch import LaunchDescription  # To create a launch description
from launch_ros.actions import Node  # Define ROS 2 nodes
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # Declare launch arguments
from launch.substitutions import LaunchConfiguration  # For launch configurations
from launch.conditions import IfCondition, UnlessCondition  # Conditional node launching
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):

    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() == 'true'

    # pkg_path = get_package_share_directory('path_planner_server')
    robot_config_dir = os.path.join(get_package_share_directory('robot_config'), 'config')

    # Find package share directory for rviz config
    pkg_share = FindPackageShare('attach_shelf').find('attach_shelf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'warehouse_rb1.rviz')

    robot_config_sim = os.path.join(robot_config_dir, 'sim.yaml')
    robot_config_real = os.path.join(robot_config_dir, 'real.yaml')

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Define static transform publisher node
    static_tf_pub = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        name='static_transform_publisher', 
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'robot_base_link', 'base_link']
    )

    # Approach service server node
    approach_service_server_node_sim = Node(
        package='attach_shelf',
        executable='approach_service_server',
        name='approach_service_server',
        output='screen', 
        parameters=[robot_config_sim], 
        condition=IfCondition(str(use_sim_time).lower())
    )
    
    approach_service_server_node_real = Node(
        package='attach_shelf',
        executable='approach_service_server',
        name='approach_service_server',
        output='screen', 
        parameters=[robot_config_real], 
        condition=UnlessCondition(str(use_sim_time).lower())
    )

    # launching other launch files with node selection
    localization_launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('localization_server').find('localization_server'),
                'launch',
                'localization.launch.py'
            )
        ),
        launch_arguments={
            'map_file': 'warehouse_map_keepout_sim.yaml',
            'run_rviz': 'False',  # Disable RViz from localization
            'run_amcl': 'True'    # Keep AMCL running
        }.items(),
        condition=IfCondition(str(use_sim_time).lower())
    )

    localization_launch_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('localization_server').find('localization_server'),
                'launch',
                'localization.launch.py'
            )
        ),
        launch_arguments={
            'map_file': 'warehouse_map_keepout_real.yaml',
            'run_rviz': 'False',
            'run_amcl': 'True'
        }.items(),
        condition=UnlessCondition(str(use_sim_time).lower())
    )

    pathplanner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('path_planner_server').find('path_planner_server'),
                'launch',
                'pathplanner.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time_str,
            'run_rviz': 'False',
            'run_pathplanner': 'True',
            'run_controller': 'True',
            'run_recoveries': 'True',
            'run_bt_nav': 'True',
            'run_filter_mask': 'True',
            'run_costmap_filter': 'True',
            'run_approach_service': 'False'
        }.items(),
    )

    return [
        rviz_node,
        static_tf_pub,
        localization_launch_sim,
        localization_launch_real,
        pathplanner_launch,
        # approach_service_server_node_sim,
        # approach_service_server_node_real
    ]

def generate_launch_description():

    use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        )

    return LaunchDescription([
        use_sim_time_cmd,
        OpaqueFunction(function=launch_setup)
    ])
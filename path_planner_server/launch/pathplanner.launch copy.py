import os  # For file and directory path operations
from ament_index_python.packages import get_package_share_directory  # Get share directory of a package
from launch import LaunchDescription  # To create a launch description
from launch_ros.actions import Node  # Define ROS 2 nodes
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # Declare launch arguments
from launch.substitutions import LaunchConfiguration  # For launch configurations
from launch.conditions import IfCondition, UnlessCondition  # Conditional node launching

def launch_setup(context, *args, **kwargs):
    
    # Get the actual value of use_sim_time and convert to boolean
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() == 'true'

    print(f'Path Planner Launch:')
    print(f'\t- use_sim_time: {use_sim_time}')

    pkg_path = get_package_share_directory('path_planner_server')
    robot_config_dir = os.path.join(get_package_share_directory('robot_config'), 'config')
    
    robot_config_sim = os.path.join(robot_config_dir, 'sim.yaml')
    robot_config_real = os.path.join(robot_config_dir, 'real.yaml')

    # File paths for simulation and real configuration files
    planner_yaml_sim = os.path.join(pkg_path, 'config', 'planner_sim.yaml')
    controller_yaml_sim = os.path.join(pkg_path, 'config', 'controller_sim.yaml')
    recoveries_yaml_sim = os.path.join(pkg_path, 'config', 'recoveries_sim.yaml')
    bt_navigator_yaml_sim = os.path.join(pkg_path, 'config', 'bt_navigator_sim.yaml')
    filters_yaml_sim = os.path.join(pkg_path, 'config', 'filters_sim.yaml')

    planner_yaml_real = os.path.join(pkg_path, 'config', 'planner_real.yaml')
    controller_yaml_real = os.path.join(pkg_path, 'config', 'controller_real.yaml')
    recoveries_yaml_real = os.path.join(pkg_path, 'config', 'recoveries_real.yaml')
    bt_navigator_yaml_real = os.path.join(pkg_path, 'config', 'bt_navigator_real.yaml')
    filters_yaml_real = os.path.join(pkg_path, 'config', 'filters_real.yaml')
    
    # Path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'pathplanning.rviz')

    # Define RViz node
    rviz_node = Node(
        package='rviz2', 
        executable='rviz2',
        name='rviz_node', 
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )
    
    # Define static transform publisher node
    static_tf_pub = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        name='static_transform_publisher', 
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'robot_base_link', 'base_link']
    )

    # Define path planner server node (simulation and real-time versions)
    planner_server_node_sim = Node(
        package='nav2_planner', 
        executable='planner_server',
        name='planner_server', 
        output='screen',
        parameters=[planner_yaml_sim],
        condition=IfCondition(str(use_sim_time).lower())
    )
    
    planner_server_node_real = Node(
        package='nav2_planner', 
        executable='planner_server',
        name='planner_server', 
        output='screen',
        parameters=[planner_yaml_real],
        condition=UnlessCondition(str(use_sim_time).lower())
    )

    # Define path planner server node (simulation and real-time versions)
    # planner_server_node_sim = Node(
    #     package='nav2_planner', 
    #     executable='planner_server',
    #     name='planner_server', 
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'yaml_filename': planner_yaml_sim
    #     }],
    #     condition=IfCondition(str(use_sim_time).lower())
    # )
    
    # planner_server_node_real = Node(
    #     package='nav2_planner', 
    #     executable='planner_server',
    #     name='planner_server', 
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'yaml_filename': planner_yaml_real
    #     }],
    #     condition=UnlessCondition(str(use_sim_time).lower())
    # )
    
    # Define path controller server node (simulation and real-time versions)
    controller_server_node_sim = Node(
        package='nav2_controller', 
        executable='controller_server',
        name='controller_server', 
        output='screen',
        parameters=[controller_yaml_sim],
        remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')],
        condition=IfCondition(str(use_sim_time).lower())
    )
    
    controller_server_node_real = Node(
        package='nav2_controller', 
        executable='controller_server',
        name='controller_server', 
        output='screen',
        parameters=[controller_yaml_real],
        remappings=[('/cmd_vel', '/cmd_vel')],
        condition=UnlessCondition(str(use_sim_time).lower())
    )
    # # Define path controller server node (simulation and real-time versions)
    # controller_server_node_sim = Node(
    #     package='nav2_controller', 
    #     executable='controller_server',
    #     name='controller_server', 
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'yaml_filename': controller_yaml_sim
    #     }],
    #     remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')],
    #     condition=IfCondition(str(use_sim_time).lower())
    # )
    
    # controller_server_node_real = Node(
    #     package='nav2_controller', 
    #     executable='controller_server',
    #     name='controller_server', 
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'yaml_filename': controller_yaml_real
    #     }],
    #     remappings=[('/cmd_vel', '/cmd_vel')],
    #     condition=UnlessCondition(str(use_sim_time).lower())
    # )


    # Define recovery behaviors server node (simulation and real-time versions)
    recoveries_server_node_sim = Node(
        package='nav2_behaviors', 
        executable='behavior_server',
        name='behavior_server', 
        output='screen',
        parameters=[recoveries_yaml_sim],
        remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')],
        condition=IfCondition(str(use_sim_time).lower())
    )
    
    recoveries_server_node_real = Node(
        package='nav2_behaviors', 
        executable='behavior_server',
        name='behavior_server', 
        output='screen',
        parameters=[recoveries_yaml_real],
        remappings=[('/cmd_vel', '/cmd_vel')],
        condition=UnlessCondition(str(use_sim_time).lower())
    )
    #  # Define recovery behaviors server node (simulation and real-time versions)
    # recoveries_server_node_sim = Node(
    #     package='nav2_behaviors', 
    #     executable='behavior_server',
    #     name='behavior_server', 
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'yaml_filename': recoveries_yaml_sim
    #     }],
    #     remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')],
    #     condition=IfCondition(str(use_sim_time).lower())
    # )
    
    # recoveries_server_node_real = Node(
    #     package='nav2_behaviors', 
    #     executable='behavior_server',
    #     name='behavior_server', 
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'yaml_filename': recoveries_yaml_real
    #     }],
    #     remappings=[('/cmd_vel', '/cmd_vel')],
    #     condition=UnlessCondition(str(use_sim_time).lower())
    # )  

    # Define behavior tree navigator node (simulation and real-time versions)
    bt_navigator_node_sim = Node(
        package='nav2_bt_navigator', 
        executable='bt_navigator',
        name='bt_navigator', 
        output='screen',
        parameters=[bt_navigator_yaml_sim],
        condition=IfCondition(str(use_sim_time).lower())
    )
    
    bt_navigator_node_real = Node(
        package='nav2_bt_navigator', 
        executable='bt_navigator',
        name='bt_navigator', 
        output='screen',
        parameters=[bt_navigator_yaml_real],
        condition=UnlessCondition(str(use_sim_time).lower())
    )

    # # Define behavior tree navigator node (simulation and real-time versions)
    # bt_navigator_node_sim = Node(
    #     package='nav2_bt_navigator', 
    #     executable='bt_navigator',
    #     name='bt_navigator', 
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'yaml_filename': bt_navigator_yaml_sim
    #     }],
    #     condition=IfCondition(str(use_sim_time).lower())
    # )
    
    # bt_navigator_node_real = Node(
    #     package='nav2_bt_navigator', 
    #     executable='bt_navigator',
    #     name='bt_navigator', 
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'yaml_filename': bt_navigator_yaml_real
    #     }],
    #     condition=UnlessCondition(str(use_sim_time).lower())
    # )

    # Filter mask server nodes
    filter_mask_server_node_sim = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[filters_yaml_sim],
        condition=IfCondition(str(use_sim_time).lower())
    )

    filter_mask_server_node_real = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[filters_yaml_real],
        condition=UnlessCondition(str(use_sim_time).lower())
    )

    # # Filter mask server nodes
    # filter_mask_server_node_sim = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='filter_mask_server',
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'yaml_filename': filters_yaml_sim
    #     }],
    #     condition=IfCondition(str(use_sim_time).lower())
    # )

    # filter_mask_server_node_real = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='filter_mask_server',
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'yaml_filename': filters_yaml_real
    #     }],
    #     condition=UnlessCondition(str(use_sim_time).lower())
    # )

    # Costmap filter info server nodes
    costmap_filter_info_server_node_sim = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[filters_yaml_sim],
        condition=IfCondition(str(use_sim_time).lower())
    )

    costmap_filter_info_server_node_real = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[filters_yaml_real],
        condition=UnlessCondition(str(use_sim_time).lower())
    )

    # Costmap filter info server nodes
    costmap_filter_info_server_node_sim = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': filters_yaml_sim
        }],
        condition=IfCondition(str(use_sim_time).lower())
    )

    costmap_filter_info_server_node_real = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': filters_yaml_real
        }],
        condition=UnlessCondition(str(use_sim_time).lower())
    )
    # Define lifecycle manager node to manage the lifecycle of all nodes
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager', 
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner', 
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator',
                'filter_mask_server',
                'costmap_filter_info_server'
            ]
        }]
    )
    
    # Approach service server nodes
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
    
    # Return all nodes
    return [
        rviz_node,
        static_tf_pub,
        planner_server_node_sim,
        planner_server_node_real,
        controller_server_node_sim,
        controller_server_node_real,
        recoveries_server_node_sim,
        recoveries_server_node_real,
        bt_navigator_node_sim,
        bt_navigator_node_real,
        filter_mask_server_node_sim,
        filter_mask_server_node_real,
        costmap_filter_info_server_node_sim,
        costmap_filter_info_server_node_real,
        lifecycle_manager_node,
        approach_service_server_node_sim,
        approach_service_server_node_real,
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
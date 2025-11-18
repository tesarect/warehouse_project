import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    
    
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ['true', '1', 'yes']

    pkg_path = get_package_share_directory('path_planner_server')

    config_suffix = 'sim' if use_sim_time else 'real'
    robot_config_dir = os.path.join(get_package_share_directory('robot_config'), 'config')
    robot_config = os.path.join(robot_config_dir, f'{config_suffix}.yaml')

    controller_yaml = os.path.join(pkg_path, 'config', f'controller_{config_suffix}.yaml')
    bt_navigator_yaml = os.path.join(pkg_path, 'config', f'bt_navigator_{config_suffix}.yaml')
    planner_yaml = os.path.join(pkg_path, 'config', f'planner_{config_suffix}.yaml')
    recovery_yaml = os.path.join(pkg_path, 'config', f'recoveries_{config_suffix}.yaml')

    controller_remappings = []
    if use_sim_time:
        controller_remappings = [('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]


    approach_service_server_node = Node(
        package='attach_shelf',
        executable='approach_service_server',
        name='approach_service_server',
        output='screen',
        parameters=[robot_config]
        # prefix='bash -c \'sleep 10; $0 $@\''
        )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml, {'use_sim_time': use_sim_time}],
        remappings=controller_remappings)

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml])

    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='recoveries_server',
        parameters=[recovery_yaml],
        output='screen')

    navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml])

    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['planner_server',
                                    'controller_server',
                                    'recoveries_server',
                                    'bt_navigator']}])

    nodes = [
        approach_service_server_node,
        controller_node,
        planner_node,
        behavior_node,
        navigator_node,
        lifecycle_node,
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),
        OpaqueFunction(function=launch_setup)
    ])

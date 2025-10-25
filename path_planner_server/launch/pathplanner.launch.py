import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):

    rviz_config = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'pathplanning.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    # pick the respective config files
    config_suffix = 'sim' if use_sim_time else 'real'

    pkg_path = get_package_share_directory('path_planner_server')

    controller_yaml = os.path.join(pkg_path, 'config', f'controller_{config_suffix}.yaml')
    bt_navigator_yaml = os.path.join(pkg_path, 'config', f'bt_navigator_{config_suffix}.yaml')
    planner_yaml = os.path.join(pkg_path, 'config', f'planner_{config_suffix}.yaml')
    recovery_yaml = os.path.join(pkg_path, 'config', f'recoveries_{config_suffix}.yaml')

    # Define controller node with conditional remapping
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml],
    )
    # Only add the remapping for simulation
    if use_sim_time:
        controller_node.remappings = [('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
  
    nodes = [

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config]), 

        controller_node,

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator']}]),
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
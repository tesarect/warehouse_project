import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # rviz_config = os.path.join(get_package_share_directory('map_server'), 'rviz', 'map_display.rviz')
    # rviz_config = os.path.join(get_package_share_directory('map_server'), 'rviz', '3rd_prsn_view.rviz')

    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')

    # rviz2                         - DONE
    # planner_server                - DONE
    # controller_server             - DONE
    # recoveries_server             - DONE
    # bt_navigator                  - DONE
    # lifecycle_manager_pathplanner - DONE

    return LaunchDescription([     

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),
       
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     output='screen',
        #     name='rviz_node',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     arguments=['-d', rviz_config]),
        
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings=[
                ('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]),

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
    ])

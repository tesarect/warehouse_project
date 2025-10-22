from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare arguments to be passed down
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Full path to the map YAML file to load'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Use LaunchConfiguration to get the runtime value
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Declare the arguments so they can be used in this launch file
        map_file_arg,
        use_sim_time_arg,

        # Include localization launch with argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('localization_server'),
                    'launch',
                    'localization.launch.py'
                ])
            ),
            launch_arguments={
                'map_file': map_file
            }.items()
        ),

        # Include cartographer launch with argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('cartographer_slam'),
                    'launch',
                    'cartographer.launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        ),
    ])

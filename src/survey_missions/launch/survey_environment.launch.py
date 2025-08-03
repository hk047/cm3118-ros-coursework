import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the config file
    config_file_path = os.path.join(
        get_package_share_directory('survey_missions'),
        'config',
        'survey_waypoints_1.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=config_file_path,
            description='Path to the waypoints file.'
        ),

        Node(
            package='survey_missions',
            executable='mission_handler',
            name='mission_handler_node',
            output='screen',
            parameters=[{'waypoints_file': LaunchConfiguration('waypoints_file')}]
        ),
    ])
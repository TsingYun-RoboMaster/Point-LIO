import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_path = LaunchConfiguration('config_path', default='')

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value='',
            description='Path to YAML config file'
        ),
        Node(
            package='point_lio',
            executable='pointlio_mapping',
            name='point_lio',
            parameters=[config_path],
            output='screen',
        ),
    ])

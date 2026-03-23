from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('gap_explorer')
    params_file = os.path.join(pkg_share, 'config', 'gap_explorer.yaml')

    return LaunchDescription([
        Node(
            package='gap_explorer',
            executable='gap_explorer',
            name='gap_explorer',
            output='screen',
            parameters=[params_file],
        )
    ])
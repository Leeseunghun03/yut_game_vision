from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_file_path = os.path.join(
        get_package_share_directory('xyz_yut_vision'),
        'config',
        'param.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=param_file_path,
            description='Full path to the parameter file to load'
        ),

        Node(
            package='xyz_yut_vision',
            executable='xyz_yut_vision_node',
            name='xyz_yut_vision_node',
            output='screen',
            parameters=[param_file_path]

        )
    ])


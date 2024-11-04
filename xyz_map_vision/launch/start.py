from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_file_path = os.path.join(
        get_package_share_directory('xyz_map_vision'),
        'config',
        'camera.yaml'
    )
    
    print(f"Loading parameters from: {param_file_path}")

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=param_file_path,
            description='Full path to the parameter file to load'
        ),

        Node(
            package='xyz_map_vision',
            executable='xyz_map_vision_node',
            name='xyz_map_vision_node',
            output='screen',
            parameters=[param_file_path]
        )
    ])


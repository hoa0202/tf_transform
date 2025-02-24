from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('tf_publisher')
    config_file = os.path.join(pkg_dir, 'config', 'tf_config.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to config file'
        ),
        
        Node(
            package='tf_publisher',
            executable='tf_publisher_node',
            name='tf_publisher',
            parameters=[{
                'config_file': LaunchConfiguration('config_file')
            }],
            output='screen'
        )
    ]) 
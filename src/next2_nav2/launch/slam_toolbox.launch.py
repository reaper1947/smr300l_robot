from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'config', 'mapper_params_online_async.yaml'
    )
    config_path = os.path.abspath(config_path)
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[config_path]
        )
    ])
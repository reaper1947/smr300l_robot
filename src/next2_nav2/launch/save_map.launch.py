from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_saver',
            name='map_saver',
            output='screen',
            parameters=[{'save_map_timeout': 2000}],
            arguments=['-f', '/home/next2/next_ros2/src/next2_nav2/config/map_02']
        )
    ])
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='next2_system',
            executable='xiao_node.py',
            name='xiao_node',
            output='screen'
        )
    ])

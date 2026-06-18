from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='next2_dynamic_parameter',
            executable='io_logic.py',
            name='bypass_node',
            output='screen'
        )
    ])

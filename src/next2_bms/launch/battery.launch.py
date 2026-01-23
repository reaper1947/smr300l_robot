from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='next2_bms',
            # executable='battery.py',
            executable='battery_can.py',
            name='greenwaybattery',
            output='screen',
            # parameters=[{'can_device': 'can1'}]
        )
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='next2_system',
            executable='diagnostics_node.py',
            name='diagnostics_node',
            output='screen'
        ),
        # Node(
        #     package='next2_io',
        #     executable='all_io.py',
        #     name='device_io',
        #     output='screen'
        # ),
        # Node(
        #     package='next2_bms',
        #     # executable='battery.py',
        #     executable='battery_can.py',
        #     name='greenwaybattery',
        #     output='screen',
        #     parameters=[{'can_device': 'can1'}]
        # ),
        # Node(
        #     package='next2_system',
        #     executable='xiao_node.py',
        #     name='xiao_node',
        #     output='screen'
        # )
    ])

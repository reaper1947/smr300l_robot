from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xbee',
            # executable='xbee_node.py',
            executable='zigbee.py',
            name='xbee_node',
            parameters=[{
                'port': '/dev/ttyUSB5',
                'send_message': 'Hello A',
                'H': 0x02,
                'F': 0x03
            }],
            output='screen'
        ),

        # Node(
        #     package='xbee',
        #     executable='xbee_node.py',
        #     name='xbee_node_b',
        #     parameters=[{
        #         'port': '/dev/ttyUSB0',
        #         'send_message': 'Hello B',
        #         'H': 0x02,
        #         'F': 0x03
        #     }],
        #     output='screen'
        # ),
    ])

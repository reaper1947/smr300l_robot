from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='next2_motor',
            executable='motor2.py',
            name='motor_controller',
            output='screen',
            parameters=[
                {"device_name": "kinco"},
                {"wheels_x_distance": 0.0},
                {"wheels_y_distance": 0.49},
                {"wheel_diameter": 0.2},
                {"max_motor_rpm": 3000.0},
                {"gear_ratio": 20.00},
                {"total_wheels": 2.0},
                {"can_bitrate": 500000},
                {"motor_1_id": 1},
                {"motor_2_id": 2}
            ]
        )
    ])

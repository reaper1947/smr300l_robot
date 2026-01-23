from launch import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import subprocess

def setup_can_interface(context, *args, **kwargs):
    try:
        result = subprocess.run("ip link show can0 | grep -q 'state UP'", shell=True)
        if result.returncode != 0:
            subprocess.run("sudo ip link set can0 down", shell=True)
            subprocess.run("sudo ip link set can0 up type can bitrate 500000", shell=True)
            subprocess.run("sudo ip link set can0 up", shell=True)
            subprocess.run("cansend can0 000#0100",shell=True)
        else:
            print("can0 already up")
    except Exception as e:
        print(f"CAN setup failed: {e}")
    return []

def generate_launch_description():
    bringup_dir        = os.path.join(os.getenv('HOME'), 'next_ros2', 'install')
    io_launch_path     = os.path.join(bringup_dir, 'next2_io', 'share', 'next2_io', 'launch', 'io.launch.py')
    bat_launch_path    = os.path.join(bringup_dir, 'next2_bms', 'share', 'next2_bms', 'launch', 'battery.launch.py')
    di_launch_path     = os.path.join(bringup_dir, 'next2_system', 'share', 'next2_system', 'launch', 'diagnostics.launch.py')
    xiao_launcg_path   = os.path.join(bringup_dir, 'next2_system', 'share', 'next2_system', 'launch', 'xiao.launch.py')
    motor_launch_path  = os.path.join(bringup_dir, 'dan_ros_motor', 'share', 'dan_ros_motor', 'launch', 'dan_motor.launch.py')
    lidar_front        = os.path.join(bringup_dir, 'sllidar_ros2', 'share', 'sllidar_ros2', 'launch', 'sllidar_t1_launch.py')
    urdf_chassis       = os.path.join(bringup_dir, 'next2_motor', 'share', 'next2_motor', 'launch', 'rsp.launch.py')
    laser_filter       = os.path.join(bringup_dir, 'laser_filters', 'share', 'laser_filters', 'examples', 'box_filter_example.launch.py')

    #node
    robot_mode = Node(package='next2_system', executable='robot_mode.py', name='mode_publisher', output='screen')
    # io_node    = Node(package='next2_io', executable='all_io.py', name='device_io', output='screen')
    dmx_node = Node(package='next2_indicator', executable='indicator_node.py', name='led_indicator', output='screen')
    # bypass = Node(package='next2_dynamic_parameter', executable='io_logic.py', name='logic_param_node', output='screen')

    # jack_node = Node(package='dan_ros_motor', executable='jack_action_server.py', name='jack_lift_server', output='screen')

    return LaunchDescription([
        OpaqueFunction(function=setup_can_interface),
        robot_mode,
        # io_node,
        # bypass,
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(io_launch_path)
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(bat_launch_path)
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(laser_filter)
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(motor_launch_path)
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(di_launch_path)
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(xiao_launcg_path)
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(lidar_front)
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(urdf_chassis)
                ),
            ]
        ),
        dmx_node,
        #jack_node
    ])

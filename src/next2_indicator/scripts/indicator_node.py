#!/usr/bin/env python3

import os
import json
import math
from math import floor

from indicator_driver import DMXDriver

import rclpy
from rclpy.node import Node
from next2_msgs.msg import RobotMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool

RUNNING = 2
EMERGENCY = 5
IDLE = 1
NO_COLOR = 0
BATTERY_COLORS = 22
BOOTING = 11
CHARGING = 15
MANUAL_CHARGE = 19
ERROR = 14
PAUSE = 23

class DMX512Node(Node):
    def __init__(self):
        super().__init__('dmx_512')

        # Color (r g b)
        self.RUNNING = (0,255,0) # green
        self.EMERGENCY = (255,0,0) # red
        self.IDLE = (0,0,255) # blue
        self.NO_COLOR = (0,0,0) # none
        self.BATTERY_COLORS = [(229,31,31), (242,161,52), (247,227,121), (187,219,68)] #green shade from 0% to full
        self.BOOTING = (128,0,128) # purple
        self.CHARGING = (255,255,0) # yellow
        self.PAUSE = (255,0,255) # pink
        
        # New safety-related colors
        self.SAFETY_STOP = (100, 100, 0) # Dark Yellow
        self.SUPER_OVERRIDE = (255, 0, 0) # Red
        self.MANUAL_OVERRIDE = (255, 255, 0) # Bright Yellow
        self.LOCALIZATION_INVALID = (0, 255, 255) # Cyan (Light Blue)

        package_share_directory = get_package_share_directory('dan_ros_motor')
        json_file_path = os.path.join(package_share_directory, 'config/motor_param.json')
        if not os.path.exists(json_file_path):
            self.get_logger().error("motor_param.json not found")
            return
            
        with open(json_file_path, 'r') as f:
            data = json.load(f)

        odom_topic_name = data['odom_topic_name']

        self.dmx = DMXDriver()

        # --- Publishers ---
        self.heartbeat_pub = self.create_publisher(Bool, 'dmx_heartbeat', 10)
        # --- Timer for Heartbeat (1 Hz) ---
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # Add a ROS2 parameter for charging dim level
        self.declare_parameter('charging_dim_level', 0.2)
        self.charging_dim_level = self.get_parameter('charging_dim_level').get_parameter_value().double_value

        # Subscribe to robot mode
        self.subscription = self.create_subscription(
            RobotMode,
            'robot_mode',
            self.mode_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic_name,
            self.odom_callback,
            10
        )

        self.battery_sub = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_callback,
            10
        )
        
        # New safety state subscribers
        self.super_override_sub = self.create_subscription(
            Bool,
            '/safety/super_override_active',
            self.super_override_callback,
            10
        )
        self.manual_override_sub = self.create_subscription(
            Bool,
            '/safety/override_active',
            self.manual_override_callback,
            10
        )
        self.obstacle_stop_sub = self.create_subscription(
            Bool,
            '/safety/obstacle_stop_active',
            self.obstacle_stop_callback,
            10
        )
        self.loc_invalid_sub = self.create_subscription(
            Bool,
            '/safety/localization_invalid',
            self.loc_invalid_callback,
            10
        )

        self.battery_percent = 1.0
        self.mode = 11
        self.last_cmd = None

        self.color = self.NO_COLOR
        self.blink = False
        self.direction = None
        
        # Safety state flags
        self.super_override_active = False
        self.manual_override_active = False
        self.safety_stop_active = False
        self.localization_invalid_active = False

        self.get_logger().info("DMX Controller Node started, expanded with safety indicators.")

    def publish_heartbeat(self):
        msg = Bool()
        msg.data = True
        self.heartbeat_pub.publish(msg)
        
    def mode_callback(self, msg):
        self.mode = msg.robot_mode

    def odom_callback(self, msg):
        cmd_l_x = msg.twist.twist.linear.x
        cmd_a_z = msg.twist.twist.angular.z

        if cmd_a_z == 0 and cmd_l_x == 0:
            self.direction = 'stay'
        elif cmd_a_z != 0:
            self.direction = 'right' if cmd_a_z < 0 else 'left'
        elif cmd_l_x != 0:
            self.direction = 'straight'

    def battery_callback(self, msg):
        self.battery_percent = msg.percentage
        
    def super_override_callback(self, msg):
        self.super_override_active = msg.data

    def manual_override_callback(self, msg):
        self.manual_override_active = msg.data
        
    def obstacle_stop_callback(self, msg):
        self.safety_stop_active = msg.data
        
    def loc_invalid_callback(self, msg):
        self.localization_invalid_active = msg.data

    def dmx_compute(self):
        # Default settings
        self.dmx.set_dim_level(1.0)
        self.dmx.set_freq(1.0) # Default blink 1.0s
        
        # --- Priority 1: Emergency Case ---
        if self.mode == EMERGENCY or self.mode == ERROR:
            self.color = self.EMERGENCY
            self.dmx.set_blink(True)
            self.dmx.set_freq(0.4) # Fast blink for emergency
            self.dmx.set_led_colour(self.color)
            return

        # --- Priority 2: Super Override (Blinking Red) ---
        if self.super_override_active:
            self.color = self.SUPER_OVERRIDE
            self.dmx.set_blink(True)
            self.dmx.set_freq(0.5) # Medium-fast blink
            self.dmx.set_led_colour(self.color)
            return

        # --- Priority 3: Localization Invalid (Blinking Cyan) ---
        if self.localization_invalid_active:
            self.color = self.LOCALIZATION_INVALID
            self.dmx.set_blink(True)
            self.dmx.set_freq(0.8)
            self.dmx.set_led_colour(self.color)
            return

        # --- Priority 4: Obstacle Safety Stop (Solid Dark Yellow) ---
        if self.safety_stop_active:
            self.color = self.SAFETY_STOP
            self.dmx.set_blink(False)
            self.dmx.set_led_colour(self.color)
            return

        # --- Priority 5: Manual Override (Blinking Yellow) ---
        if self.manual_override_active:
            self.color = self.MANUAL_OVERRIDE
            self.dmx.set_blink(True)
            self.dmx.set_freq(1.0)
            self.dmx.set_led_colour(self.color)
            return

        # --- Priority 6: Normal Operating Modes ---
        if self.mode == RUNNING:
            # If mode is RUNNING but robot is static, show battery percentage
            if self.direction == 'stay':
                self.dmx.set_blink(False)
                colors = self.BATTERY_COLORS
                if self.battery_percent >= 1.0:  # Full
                    self.dmx.set_led_colour(colors[3])
                else:
                    idx = floor(self.battery_percent * 4)
                    idx = max(0, min(idx, 3))
                    self.dmx.set_led_colour(colors[idx])
            else:
                self.dmx.set_blink(True)
                self.dmx.set_freq(0.7) # Faster blink when moving
                self.color = self.RUNNING
                if self.direction == 'right':
                    start, end = 3, 4
                elif self.direction == 'left':
                    start, end = 0, 1
                else:
                    start, end = 0, 4
                self.dmx.set_led_colour(self.color, start=start, end=end)

        elif self.mode == IDLE:
            self.color = self.IDLE
            self.dmx.set_blink(False)
            self.dmx.set_led_colour(self.color)

        elif self.mode == BOOTING:
            self.color = self.BOOTING
            self.dmx.set_blink(True)
            self.dmx.set_freq(1.2) # Slow blink for booting
            self.dmx.set_led_colour(self.color)

        elif self.mode == CHARGING or self.mode == MANUAL_CHARGE:
            self.color = self.CHARGING
            self.dmx.set_blink(True)
            self.dmx.set_freq(1.5) # Very slow pulse-like blink for charging
            self.dmx.set_dim_level(self.charging_dim_level)
            self.dmx.set_led_colour(self.color, dim=True)

        elif self.mode == PAUSE:
            self.color = self.PAUSE
            self.dmx.set_blink(False)
            self.dmx.set_led_colour(self.color)

        else:
            self.color = self.NO_COLOR
            self.dmx.set_blink(False)
            self.dmx.set_led_colour(self.color)

    def run(self):
        try:
            while rclpy.ok():
                self.dmx_compute()
                self.dmx.update_dmx()
                rclpy.spin_once(self, timeout_sec=0.05)
        except KeyboardInterrupt:
            pass
        finally:
            self.dmx.destroy_dmx()


def main(args=None):
    rclpy.init(args=args)
    dmx_node = DMX512Node()
    dmx_node.run()
    dmx_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

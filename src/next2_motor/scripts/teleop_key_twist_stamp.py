#!/usr/bin/env python3

# Copyright (c) 2011, Willow Garage, Inc.
# Licensed under the BSD License

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

MAX_LIN_VEL = 1.2
MAX_ANG_VEL = 1.0

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your NextRobot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (NextRobot : ~ 1.2)
a/d : increase/decrease angular velocity (NextRobot : ~ 1.0)
space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(input, low, high):
    return max(min(input, high), low)

class TeleopNode(Node):
    def __init__(self):
        super().__init__('NextRobot_teleop')
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0

    def update_TwistStamped(self):
        twist_stamped = TwistStamped()  # แก้ชื่อ object เป็นตัวพิมพ์เล็ก
        twist_stamped.header.stamp = self.get_clock().now().to_msg()  # เพิ่ม header.stamp
        twist_stamped.header.frame_id = ''  # frame_id ว่างสำหรับกรณีทั่วไป

        self.control_linear_vel = self.makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, LIN_VEL_STEP_SIZE / 2.0)
        twist_stamped.twist.linear.x = self.control_linear_vel
        twist_stamped.twist.linear.y = 0.0
        twist_stamped.twist.linear.z = 0.0

        self.control_angular_vel = self.makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, ANG_VEL_STEP_SIZE / 2.0)
        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0
        twist_stamped.twist.angular.z = self.control_angular_vel

        self.pub.publish(twist_stamped)

    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min(input, output + slop)
        elif input < output:
            output = max(input, output - slop)
        return output

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    print(msg)

    try:
        while True:
            key = getKey(settings if os.name != 'nt' else None)
            if key == 'w':
                node.target_linear_vel = constrain(node.target_linear_vel + LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
            elif key == 'x':
                node.target_linear_vel = constrain(node.target_linear_vel - LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
            elif key == 'a':
                node.target_angular_vel = constrain(node.target_angular_vel + ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL)
            elif key == 'd':
                node.target_angular_vel = constrain(node.target_angular_vel - ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL)
            elif key in (' ', 's'):
                node.target_linear_vel = 0.0
                node.target_angular_vel = 0.0
            elif key == '\x03':  # CTRL-C
                break

            node.update_TwistStamped()

    except Exception as ex:
        node.get_logger().error(f"Error: {str(ex)}")
    finally:
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

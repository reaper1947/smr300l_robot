#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, os
import time

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

# ---------------- PARAMETERS ----------------
MAX_LIN_VEL = 1.2
MAX_ANG_VEL = 1.0

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

PUBLISH_RATE = 0.05      # 20 Hz
DEADMAN_TIMEOUT = 0.5   # seconds

msg = """
Control Your NextRobot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space, s : force stop

CTRL-C to quit
"""

# ---------------- UTILITIES ----------------
def getKey(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(value, low, high):
    return max(min(value, high), low)

# ---------------- TELEOP NODE ----------------
class TeleopNode(Node):
    def __init__(self):
        super().__init__('nextrobot_teleop')

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0

        self.last_key_time = time.time()

        # Timers
        self.create_timer(PUBLISH_RATE, self.publish_twist)
        self.create_timer(0.1, self.deadman_check)

    def publish_twist(self):
        twist = Twist()

        self.current_linear = self.smooth(
            self.current_linear,
            self.target_linear,
            LIN_VEL_STEP_SIZE / 2.0
        )

        self.current_angular = self.smooth(
            self.current_angular,
            self.target_angular,
            ANG_VEL_STEP_SIZE / 2.0
        )

        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular

        self.pub.publish(twist)

    def smooth(self, current, target, step):
        if target > current:
            return min(target, current + step)
        elif target < current:
            return max(target, current - step)
        return current

    def deadman_check(self):
        if time.time() - self.last_key_time > DEADMAN_TIMEOUT:
            self.target_linear = 0.0
            self.target_angular = 0.0

# ---------------- MAIN ----------------
def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    print(msg)

    try:
        while rclpy.ok():
            key = getKey(settings if os.name != 'nt' else None)

            if key == 'w':
                node.target_linear = constrain(
                    node.target_linear + LIN_VEL_STEP_SIZE,
                    -MAX_LIN_VEL, MAX_LIN_VEL
                )
            elif key == 'x':
                node.target_linear = constrain(
                    node.target_linear - LIN_VEL_STEP_SIZE,
                    -MAX_LIN_VEL, MAX_LIN_VEL
                )
            elif key == 'a':
                node.target_angular = constrain(
                    node.target_angular + ANG_VEL_STEP_SIZE,
                    -MAX_ANG_VEL, MAX_ANG_VEL
                )
            elif key == 'd':
                node.target_angular = constrain(
                    node.target_angular - ANG_VEL_STEP_SIZE,
                    -MAX_ANG_VEL, MAX_ANG_VEL
                )
            elif key in (' ', 's'):
                node.target_linear = 0.0
                node.target_angular = 0.0
            elif key == '\x03':  # CTRL-C
                break

            if key:
                node.last_key_time = time.time()

            rclpy.spin_once(node, timeout_sec=0.01)

    finally:
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

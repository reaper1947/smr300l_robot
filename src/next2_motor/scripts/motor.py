#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import can
import time
import math

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # ---------------- ROS ----------------
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

        # ---------------- CAN ----------------
        self.can_bus = can.Bus(
            channel='can0',
            interface='socketcan',
            bitrate=500000
        )

        # ---------------- Robot params ----------------
        self.max_rpm = 30
        self.gear_ratio = 2
        self.wheels_x_distance_ = 0.0
        self.wheels_y_distance_ = 0.5
        self.wheel_diameter_ = 0.2
        self.wheel_circumference_ = math.pi * self.wheel_diameter_

        # ---------------- State ----------------
        self.left_rpm = 0
        self.right_rpm = 0
        self.last_cmd_time = time.time()

        # ---------------- Timers ----------------
        self.create_timer(0.05, self.can_send_loop)   # 20 Hz CAN output
        self.create_timer(0.1, self.watchdog)         # Deadman check

        # ---------------- Init motor ----------------
        self.device_name = "kinco"
        self.init_motor()

    # =================================================
    # Motor initialization (Kinco CANopen)
    # =================================================
    def init_motor(self):
        self.get_logger().info("Initializing motor...")
        try:
            commands = [
                [0x2b, 0x0f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00],
                [0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00],
                [0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00],
                [0x2b, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00],
                [0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00],
            ]

            for command in commands:
                self.can_bus.send(
                    can.Message(arbitration_id=0x601, data=command, is_extended_id=False)
                )
                self.can_bus.send(
                    can.Message(arbitration_id=0x602, data=command, is_extended_id=False)
                )
                time.sleep(0.1)

            self.get_logger().info("Motor initialized successfully")

        except can.CanError as e:
            self.get_logger().error(f"Motor init failed: {e}")

    # =================================================
    # ROS callback (NO CAN SENDING HERE)
    # =================================================
    def cmd_vel_callback(self, msg):
        self.left_rpm, self.right_rpm = self.calculate_rpm(
            msg.linear.x, msg.angular.z
        )
        self.last_cmd_time = time.time()

    # =================================================
    # RPM calculation
    # =================================================
    def calculate_rpm(self, linear_x, angular_z):
        linear_vel_mins = linear_x * 60
        angular_vel_mins = -angular_z * 60

        tangential_vel = angular_vel_mins * (
            (self.wheels_x_distance_ / 2) + (self.wheels_y_distance_ / 2)
        )

        x_rpm = linear_vel_mins / self.wheel_circumference_
        tan_rpm = tangential_vel / self.wheel_circumference_

        left = (x_rpm - tan_rpm) * self.gear_ratio
        right = -(x_rpm + tan_rpm) * self.gear_ratio

        left = max(-self.max_rpm, min(self.max_rpm, left))
        right = max(-self.max_rpm, min(self.max_rpm, right))

        return int(left), int(right)

    # =================================================
    # CAN sending loop (20 Hz)
    # =================================================
    def can_send_loop(self):
        try:
            left_msg = can.Message(
                arbitration_id=0x601,
                data=[0x23, 0xFF, 0x60, 0x00, 0, 0,
                      self.left_rpm & 0xFF,
                      (self.left_rpm >> 8) & 0xFF],
                is_extended_id=False
            )

            right_msg = can.Message(
                arbitration_id=0x602,
                data=[0x23, 0xFF, 0x60, 0x00, 0, 0,
                      self.right_rpm & 0xFF,
                      (self.right_rpm >> 8) & 0xFF],
                is_extended_id=False
            )

            self.can_bus.send(left_msg)
            self.can_bus.send(right_msg)

        except can.CanError:
            self.get_logger().warn("CAN TX buffer full")

    # =================================================
    # Deadman watchdog (stop if cmd_vel lost)
    # =================================================
    def watchdog(self):
        if time.time() - self.last_cmd_time > 0.5:
            self.left_rpm = 0
            self.right_rpm = 0

# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = MotorController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, TransformStamped
from nav_msgs.msg import Odometry

from std_srvs.srv import SetBool
import tf_transformations
from math import sin, cos, pi
from struct import unpack
import can

from tf2_ros import TransformBroadcaster

class KalmanFilter:
    def __init__(self, process_noise, measurement_noise):
        self.Q = process_noise
        self.R = measurement_noise
        self.x = 0.0
        self.P = 1.0

    def filter(self, measurement):
        self.P += self.Q
        K = self.P / (self.P + self.R)
        self.x += K * (measurement - self.x)
        self.P *= (1 - K)
        return self.x

class UpdateOdom(Node):
    def __init__(self):
        super().__init__("kinco")

        # Parameter declaration
        self.declare_parameters(
            namespace='',
            parameters=[
                ("can_device", "can0"),
                ("canbus_bitrate", 500000),
                ("motor_1_id", 1),
                ("motor_2_id", 2),
                ("velocity_unit_rpm", 1),
                ("wheels_x_distance", 0.0),
                ("wheels_y_distance", 0.49),
                ("wheel_diameter", 0.2),
                ("max_motor_rpm", 3000.0),
                ("gear_ratio", 20.00),
                ("total_wheels", 2.0),
                ("publish_tf", True),
                ("base_frame", "base_link"),
                ("odom_frame", "odom"),
            ]
        )

        # Parameters
        self.can_device = self.get_parameter("can_device").value
        self.can_bitrate = self.get_parameter("canbus_bitrate").value
        self.motor_1_id = self.get_parameter("motor_1_id").value
        self.motor_2_id = self.get_parameter("motor_2_id").value
        self.velocity_unit_rpm = self.get_parameter("velocity_unit_rpm").value
        self.wheels_x_distance = self.get_parameter("wheels_x_distance").value
        self.wheels_y_distance = self.get_parameter("wheels_y_distance").value
        self.wheel_diameter = self.get_parameter("wheel_diameter").value
        self.max_motor_rpm = self.get_parameter("max_motor_rpm").value
        self.gear_ratio = self.get_parameter("gear_ratio").value
        self.total_wheels = self.get_parameter("total_wheels").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value

        self.wheel_circumference = pi * self.wheel_diameter

        # CAN bus setup
        filters = [
            {"can_id": 0x181, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x182, "can_mask": 0x7FF, "extended": False},
        ]
        self.bus = can.interface.Bus(interface='socketcan', channel=self.can_device, can_filters=filters)

        # Publishers, subscribers, and services
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.reset_service = self.create_service(SetBool, "/reset_odom", self.callback_reset_odom)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.01, self.calculate_odom)

    def callback_reset_odom(self, request, response):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.m1 = 0.00
        self.m2 = 0.00
        response.success = True
        response.message = "Odom reset success!"
        return response

    def get_rpm(self):
        rpm1, rpm2 = 0, 0
        received_rpm1 = False
        received_rpm2 = False
        timeout = self.get_clock().now() + rclpy.duration.Duration(seconds=1)

        while self.get_clock().now() < timeout:
            frame = self.bus.recv(timeout=0.05)
            if frame is None:
                self.get_logger().warn("Timeout waiting for CAN message")
                break
            if frame.arbitration_id == 0x181:
                try:
                    m1 = unpack('<i', frame.data[0:4])[0]
                    rpm1 = (m1 * 1875 / (512 * 2500 * 4)) * self.velocity_unit_rpm
                    print(f"[get_rpm]: rpm1 {rpm1}")
                    # if m1 >= 10:

                    received_rpm1 = True
                except Exception as e:
                    self.get_logger().error(f"Error parsing frame data: {e}")
            if frame.arbitration_id == 0x182:
                try:
                    m2 = unpack('<i', frame.data[0:4])[0]
                    rpm2 = (m2 * 1875 / (512 * 2500 * 4)) * self.velocity_unit_rpm
                    # print(f"[get_rpm]: rpm2 {rpm2}")

                    received_rpm2 = True
                except Exception as e:
                    self.get_logger().error(f"Error parsing frame data: {e}")
            if received_rpm1 and received_rpm2:
                break
        return rpm1 / self.gear_ratio, rpm2 / self.gear_ratio

    def get_velocities(self, rpm1, rpm2):
        avg_rps_x = ((float)(rpm1 - rpm2) / self.total_wheels) / 60.0
        vel_x = avg_rps_x * self.wheel_circumference
        vel_y = 0.0

        avg_rps_a = ((float)(rpm1 + rpm2) / self.total_wheels) / 60.0
        ang_z = (avg_rps_a * self.wheel_circumference) / ((self.wheels_x_distance / 2) + (self.wheels_y_distance / 2))
        ang_z = -ang_z
        return vel_x, vel_y, ang_z
    def calculate_odom(self):
        rpm1, rpm2 = self.get_rpm()
        self.vx, self.vy, self.vth = self.get_velocities(rpm1, rpm2)

        # Filtering small velocities (deadband logic)
        self.vx = 0.00 if abs(self.vx) <= 0.06 else self.vx
        self.vth = 0.00 if abs(self.vth) <= 0.1 else self.vth * -1

        self.current_time = self.get_clock().now()
        dt = (self.current_time - self.last_time).nanoseconds * 1e-9
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        odom_quat = tf_transformations.quaternion_from_euler(0, 0, self.th)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = self.current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame

            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = odom_quat[0]
            t.transform.rotation.y = odom_quat[1]
            t.transform.rotation.z = odom_quat[2]
            t.transform.rotation.w = odom_quat[3]

            self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = self.current_time.to_msg()
        odom.header.frame_id = self.odom_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        odom.child_frame_id = self.base_frame
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vth

        self.odom_pub.publish(odom)

        # Add Pose2D for 2D pose tracking
        pose2d = Pose2D()
        pose2d.x = self.x
        pose2d.y = self.y
        pose2d.theta = self.th

        self.last_time = self.current_time

def main(args=None):
    rclpy.init(args=args)
    node = UpdateOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

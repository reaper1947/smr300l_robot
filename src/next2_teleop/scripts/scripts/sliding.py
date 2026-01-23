import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math
import can
from struct import unpack
from math import sin, cos, pi
class SlidingModeController(Node):
    def __init__(self):
        super().__init__('sliding_mode_controller')

        # Parameters for SMC
        self.j_ = 0.01  # Moment of inertia
        self.b_ = 0.01  # Friction coefficient
        self.lambda_ = 1.0  # Convergence rate for sliding surface
        self.k_ = 0.5  # Sliding control gain
        self.wheel_radius_ = 0.05
        self.wheel_base_ = 0.2  # Distance between wheels

        # Desired velocities (initialized to zero)
        self.left_desired_velocity_ = 0.0
        self.right_desired_velocity_ = 0.0
        self.prev_left_velocity_ = 0.0
        self.prev_right_velocity_ = 0.0
        self.prev_time_ = self.get_clock().now()

        # Subscribers
        self.cmd_vel_sub_ = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # self.joint_state_sub_ = self.create_subscription(
        #     JointState,
        #     'joint_states',
        #     self.joint_state_callback,
        #     10
        # )

        # Publishers for torque commands
        self.left_torque_pub_ = self.create_publisher(Float64, '/left_wheel/effort_command', 1)
        self.right_torque_pub_ = self.create_publisher(Float64, '/right_wheel/effort_command', 1)
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
        self.control_timer = self.create_timer(0.02, self.control_callback)

        # self.timer = self.create_timer(0.01, self.get_rpm)
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

    def cmd_vel_callback(self, msg):
        # Calculate desired wheel velocities using differential drive kinematics
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        self.left_desired_velocity_ = (linear_velocity - angular_velocity * self.wheel_base_ / 2.0) / self.wheel_radius_
        self.right_desired_velocity_ = (linear_velocity + angular_velocity * self.wheel_base_ / 2.0) / self.wheel_radius_

    def control_callback(self):
        # Read RPM from CAN
        rpm1, rpm2 = self.get_rpm()
        left_actual_velocity = (rpm1 * 2 * pi) / 60.0  # rad/s
        right_actual_velocity = (rpm2 * 2 * pi) / 60.0

        now = self.get_clock().now()
        dt = (now - self.prev_time_).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-6

        left_accel = (self.left_desired_velocity_ - self.prev_left_velocity_) / dt
        right_accel = (self.right_desired_velocity_ - self.prev_right_velocity_) / dt

        left_torque = self.calculate_smc_torque(self.left_desired_velocity_, left_actual_velocity, left_accel)
        right_torque = self.calculate_smc_torque(self.right_desired_velocity_, right_actual_velocity, right_accel)

        self.publish_torque(self.left_torque_pub_, left_torque)
        self.publish_torque(self.right_torque_pub_, right_torque)

        self.prev_left_velocity_ = self.left_desired_velocity_
        self.prev_right_velocity_ = self.right_desired_velocity_
        self.prev_time_ = now



    def calculate_smc_torque(self, desired_velocity, actual_velocity, desired_accel):
        # Calculate the error and sliding surface
        error = desired_velocity - actual_velocity
        sliding_surface = error * self.lambda_
        # Equivalent control term
        u_eq = self.j_ * desired_accel + self.b_ * actual_velocity

        # Switching control term
        u_sw = -self.k_ * math.copysign(1.0, sliding_surface)

        # Total torque command
        return u_eq + u_sw

    def publish_torque(self, publisher, torque):
        torque_msg = Float64()
        torque_msg.data = torque
        publisher.publish(torque_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SlidingModeController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

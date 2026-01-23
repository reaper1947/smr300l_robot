#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np
import math

class SlidingController(Node):
    def __init__(self):
        super().__init__('sliding_controller')

        # Parameters
        self.declare_parameter('k1', 2.0)  # Sliding surface gain
        self.declare_parameter('k2', 1.0)  # Control gain
        self.declare_parameter('lambda', 0.5)  # Sliding surface parameter
        self.declare_parameter('max_control', 1.0)  # Maximum control input
        self.declare_parameter('min_velocity', 0.1)  # Minimum velocity threshold
        self.declare_parameter('acceleration_limit', 0.5)  # Maximum acceleration

        # Get parameters
        self.k1 = self.get_parameter('k1').value
        self.k2 = self.get_parameter('k2').value
        self.lambda_param = self.get_parameter('lambda').value
        self.max_control = self.get_parameter('max_control').value
        self.min_velocity = self.get_parameter('min_velocity').value
        self.acceleration_limit = self.get_parameter('acceleration_limit').value

        # State variables
        self.current_velocity = 0.0
        self.target_velocity = 0.0
        self.last_control_input = 0.0
        self.last_time = self.get_clock().now()

        # Error variables
        self.velocity_error = 0.0
        self.integral_error = 0.0

        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.current_vel_sub = self.create_subscription(
            Float64,
            '/current_velocity',
            self.current_vel_callback,
            10)

        # Create publishers
        self.control_pub = self.create_publisher(
            Float64,
            '/control_input',
            10)

        # Create timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz

        self.get_logger().info('Sliding Controller Node has been initialized')

    def cmd_vel_callback(self, msg):
        """Callback for receiving velocity commands"""
        # Apply acceleration limit
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Limit acceleration
        max_velocity_change = self.acceleration_limit * dt
        velocity_change = msg.linear.x - self.target_velocity
        if abs(velocity_change) > max_velocity_change:
            velocity_change = np.sign(velocity_change) * max_velocity_change

        self.target_velocity += velocity_change

        # Apply minimum velocity threshold
        if abs(self.target_velocity) < self.min_velocity and abs(self.target_velocity) > 0:
            self.target_velocity = np.sign(self.target_velocity) * self.min_velocity

        self.get_logger().debug(f'Target velocity: {self.target_velocity:.3f}')

    def current_vel_callback(self, msg):
        """Callback for receiving current velocity feedback"""
        self.current_velocity = msg.data
        self.get_logger().debug(f'Current velocity: {self.current_velocity:.3f}')

    def calculate_sliding_surface(self):
        """Calculate the sliding surface"""
        # Calculate velocity error
        self.velocity_error = self.target_velocity - self.current_velocity

        # Update integral error with anti-windup
        self.integral_error += self.velocity_error * 0.01  # 0.01 is the control period
        self.integral_error = np.clip(self.integral_error, -self.max_control, self.max_control)

        # Sliding surface: s = lambda*e + e_dot + ki*integral_error
        sliding_surface = (self.lambda_param * self.velocity_error +
                         self.velocity_error / 0.01 +  # Approximate derivative
                         0.1 * self.integral_error)  # Integral term

        return sliding_surface

    def calculate_control_input(self, sliding_surface):
        """Calculate the control input using sliding mode control"""
        # Control law: u = -k1*s - k2*sign(s)
        control_input = -self.k1 * sliding_surface - self.k2 * np.sign(sliding_surface)

        # Add feedforward term for better tracking
        feedforward = 0.8 * self.target_velocity  # Feedforward gain
        control_input += feedforward

        # Saturate control input
        control_input = np.clip(control_input, -self.max_control, self.max_control)

        # Rate limit the control input
        max_change = 0.1  # Maximum change per control cycle
        control_change = control_input - self.last_control_input
        if abs(control_change) > max_change:
            control_input = self.last_control_input + np.sign(control_change) * max_change

        self.last_control_input = control_input

        return control_input

    def control_loop(self):
        """Main control loop"""
        try:
            # Calculate sliding surface
            sliding_surface = self.calculate_sliding_surface()

            # Calculate control input
            control_input = self.calculate_control_input(sliding_surface)

            # Publish control input
            control_msg = Float64()
            control_msg.data = float(control_input)
            self.control_pub.publish(control_msg)

            # Log control information
            self.get_logger().debug(
                f'Sliding surface: {sliding_surface:.3f}, '
                f'Control input: {control_input:.3f}, '
                f'Velocity error: {self.velocity_error:.3f}'
            )

        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')

def main(args=None):
    rclpy.init(args=args)

    sliding_controller = SlidingController()

    try:
        rclpy.spin(sliding_controller)
    except KeyboardInterrupt:
        pass
    finally:
        sliding_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
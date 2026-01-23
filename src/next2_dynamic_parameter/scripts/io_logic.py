#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String, Bool
from next2_msgs.msg import SeerRobotIOStatus
import csv
from datetime import datetime
import os

class LogicParamNode(Node):
    def __init__(self):
        super().__init__('logic_param_node')
        self.emer = 1
        self.DO_config = ''

        self.ry_names = [f'RY{i}' for i in range(1, 5)]
        # Declare parameters with default values
        self.declare_parameter('connected', False)
        self.declare_parameter('io_rqt', False)
        self.declare_parameter('io_topic', 'unknown')

        # Publisher
        self.relay_pub = self.create_publisher(String, 'command_rqt', 10)

        # Subscribers
        self.create_subscription(Bool, 'io_heartbeat', self.check_enable_callback, 10)
        self.create_subscription(SeerRobotIOStatus, 'status_io', self.io_status, 10)
        
        # Timestamp for last message
        self.last_io_msg_time = self.get_clock().now()
        self.last_check_enable_time = self.get_clock().now()
        
        # Timer to detect timeout on /check_enable
        self.check_timer = self.create_timer(0.05, self.check_enable_timeout) #20 hz

        # Callback for dynamic parameter updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Prepare CSV file path and open for appending
        # self.csv_path = '/home/next2/next_ros2/src/next2_log/device/io/log.csv'
        # os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        # self.csv_file = open(self.csv_path, mode='a', newline='')
        # self.csv_writer = csv.writer(self.csv_file)
        # if os.stat(self.csv_path).st_size == 0:
        #     self.csv_writer.writerow(['date_time', 'device_Status', 'DO_config'])

    # def log_connection_status(self, status):
    #     try:
    #         timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    #         self.csv_writer.writerow([timestamp, status, str(self.DO_config)])
    #         self.csv_file.flush()
    #     except Exception as e:
    #         self.get_logger().warn(f"Failed to write log: {e}")

    def io_status(self, msg):
        self.last_io_msg_time = self.get_clock().now()
        self.emer = 1  # default safe
        for io_in in msg.io_inputs:
            if io_in.id == 1:
                self.emer = io_in.status
                # print(self.emer)
                break
        self.DO_config = ''.join(str(io_out.status) for io_out in msg.io_outputs)

    def check_enable_callback(self, msg):
        self.last_check_enable_time = self.get_clock().now()

    def check_enable_timeout(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_check_enable_time).nanoseconds / 1e9
        if elapsed > 1.0:
            self.get_logger().error("Disenable!!! check io device")
            self.set_parameters([Parameter('connected', Parameter.Type.BOOL, False)])
            self.set_parameters([Parameter('io_rqt', Parameter.Type.BOOL, False)])
            # self.log_connection_status("Disconnected")
        else:
            self.set_parameters([Parameter('connected', Parameter.Type.BOOL, True)])
            now_io = self.get_clock().now()
            elapsed_io = (now_io - self.last_io_msg_time).nanoseconds / 1e9
            if elapsed_io > 1.0 or self.emer == 0:
                # self.log_connection_status("Re-Connected")
                self.get_logger().error("PLS Reconnect!!! IO don't have msg")
            # else:
                # self.log_connection_status("Connected")

    def parameter_callback(self, params):
        enable = self.get_parameter('connected').get_parameter_value().bool_value
        for param in params:
            if not enable and param.name in ['io_rqt']:
                self.get_logger().info(f"Ignoring '{param.name}' update because connected is False")
                continue
            if param.name == 'connected' and param.type_ == Parameter.Type.BOOL:
                if not param.value:
                    self.set_parameters([Parameter('io_topic', Parameter.Type.STRING, 'unknown')])
                continue
            if param.name == 'io_rqt' and param.type_ == Parameter.Type.BOOL:
                msg = String()
                msg.data = 'relay on' if param.value else 'relay off'
                self.relay_pub.publish(msg)

                io_topic = 'status_io' if param.value else 'unknown'
                self.set_parameters([Parameter('io_topic', Parameter.Type.STRING, io_topic)])

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = LogicParamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

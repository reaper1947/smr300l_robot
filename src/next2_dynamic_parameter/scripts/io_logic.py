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
        self.declare_parameter('bypass', True)
        self.declare_parameter('io_topic', 'unknown')

        # Publisher
        self.relay_pub = self.create_publisher(String, 'command_rqt', 10)

        # Subscribers
        self.create_subscription(Bool, 'io_heartbeat', self.check_enable_callback, 10)
        self.create_subscription(SeerRobotIOStatus, 'status_io', self.io_status, 10)
        
        # Timestamp for last message
        self.last_io_msg_time = self.get_clock().now()
        self.last_check_enable_time = self.get_clock().now()
        
        # Timer to detect timeout on /check_enable (20 Hz)
        self.check_timer = self.create_timer(0.05, self.check_enable_timeout) 

        # Callback for dynamic parameter updates
        self.add_on_set_parameters_callback(self.parameter_callback)

    def io_status(self, msg):
        self.last_io_msg_time = self.get_clock().now()
        # เช็ค DI ช่องที่ 1 ว่าเป็น Emergency หรือไม่ (1=ปกติ, 0=หยุดฉุกเฉิน)
        for io_in in msg.io_inputs:
            if io_in.id == 1:
                self.emer = io_in.status
                break
        self.DO_config = ''.join(str(io_out.status) for io_out in msg.io_outputs)

    def check_enable_callback(self, msg):
        self.last_check_enable_time = self.get_clock().now()

    def check_enable_timeout(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_check_enable_time).nanoseconds / 1e9
        
        # ดึงสถานะปัจจุบันของ Parameter 'connected'
        current_connected = self.get_parameter('connected').get_parameter_value().bool_value

        if elapsed > 1.0:
            # กรณีสายหลุด หรือ Node Hardware ตาย
            if current_connected:
                self.get_logger().error("Disconnected!!! check io device")
                self.set_parameters([
                    Parameter('connected', Parameter.Type.BOOL, False),
                    Parameter('bypass', Parameter.Type.BOOL, False)
                ])
        else:
            # กรณีเพิ่งกลับมาเชื่อมต่อได้ (Edge Trigger)
            if not current_connected:
                # ตรวจสอบความปลอดภัย: ต้องไม่ใช่สถานะ Emergency ถึงจะ Auto-True
                if self.emer == 1:
                    self.get_logger().info("System Re-connected! Auto-enabling Relay.")
                    self.set_parameters([
                        Parameter('connected', Parameter.Type.BOOL, True),
                        Parameter('bypass', Parameter.Type.BOOL, True)
                    ])
                else:
                    self.get_logger().warn("Re-connected but Emergency Pressed! Waiting for Reset.")
                    self.set_parameters([Parameter('connected', Parameter.Type.BOOL, True)])
            
            # ตรวจสอบสถานะ Data Message ว่ามีส่งมาไหม
            now_io = self.get_clock().now()
            elapsed_io = (now_io - self.last_io_msg_time).nanoseconds / 1e9
            if elapsed_io > 1.0:
                self.get_logger().error("Warning: Serial Connected but No Data Message!")
            elif self.emer == 0:
                self.get_logger().error("EMERGENCY STOP PRESSED")

    def parameter_callback(self, params):
        # ดึงค่า connected ล่าสุดมาเช็คก่อนอนุญาตให้เปลี่ยน bypass
        is_ready = self.get_parameter('connected').get_parameter_value().bool_value
        
        for param in params:
            # ถ้าเครื่องไม่ได้ต่ออยู่ ห้ามสั่งเปิด Relay
            if not is_ready and param.name == 'bypass':
                self.get_logger().warn(f"Cannot change '{param.name}' because device is disconnected")
                continue
            
            # เมื่อมีการเปลี่ยนสถานะ connected
            if param.name == 'connected' and param.type_ == Parameter.Type.BOOL:
                if not param.value:
                    self.set_parameters([Parameter('io_topic', Parameter.Type.STRING, 'unknown')])
                continue
            
            # เมื่อมีการเปลี่ยนค่า bypass ให้ส่ง String ไปยัง Device Node
            if param.name == 'bypass' and param.type_ == Parameter.Type.BOOL:
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
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import serial
import time


class XbeeNode(Node):
    def __init__(self):
        super().__init__('xbee_node')

        # Parameters
        self.declare_parameter('port', '/dev/xbee')
        self.declare_parameter('H', 0x02)
        self.declare_parameter('F', 0x03)
        self.declare_parameter('send_message', 'Hello com')

        self.serial_port = self.get_parameter('port').get_parameter_value().string_value
        self.send_message = self.get_parameter('send_message').get_parameter_value().string_value
        self.h = self.get_parameter('H').get_parameter_value().integer_value
        self.f = self.get_parameter('F').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(self.serial_port, 115200, timeout=0.1)
            self.get_logger().info(f"Opened port {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open port {self.serial_port}: {e}")
            self.ser = None

        # Publishers
        self.raw_pub = self.create_publisher(String, 'xbee/raw', 10)
        self.filtered_pub = self.create_publisher(String, 'xbee/filter', 10)
        self.heartbeat_pub = self.create_publisher(Bool, 'xbee/heart_beat', 10)

        # Internal buffers and timers
        self.buffer = bytearray()
        self.frame_start_time = None

        # Timers
        self.create_timer(0.1, self.send_data)       
        self.create_timer(0.1, self.send_heartbeat)  
        self.create_timer(0.1, self.read_serial) 

    def send_data(self):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warning("Serial port not open")
            return
        framed = bytes([self.h]) + self.send_message.encode() + bytes([self.f])
        self.ser.write(framed)
        self.get_logger().info(f"Sent: {self.send_message}")

    def send_heartbeat(self):
        msg = Bool()
        msg.data = True
        self.heartbeat_pub.publish(msg)

    def read_serial(self):
        if not self.ser or not self.ser.is_open:
            return

        bytes_waiting = self.ser.in_waiting # wait data in serial if have data read data
        if bytes_waiting > 0:
            data = self.ser.read(bytes_waiting)
            self.buffer += data

            raw_msg = String()
            raw_msg.data = data.decode(errors='ignore')
            self.raw_pub.publish(raw_msg)

        while True:
            start = self.buffer.find(bytes([self.h]))
            if start == -1:
                self.buffer.clear()
                self.frame_start_time = None
                break

            if self.frame_start_time is None:
                self.frame_start_time = time.time()

            end = self.buffer.find(bytes([self.f]), start + 1)
            if end == -1:
                if time.time() - self.frame_start_time > 1.0: # check footer in 1 sec if not have wait new data is have footer 03
                    self.get_logger().warn("Frame timeout: discarding incomplete data")
                    self.buffer.clear()
                    self.frame_start_time = None
                else:
                    self.buffer = self.buffer[start:]
                break

            # Valid frame
            payload_bytes = self.buffer[start + 1:end]
            self.buffer = self.buffer[end + 1:]
            self.frame_start_time = None

            try:
                payload_str = payload_bytes.decode(errors='ignore')
            except Exception as e:
                self.get_logger().warn(f"Decode error: {e}")
                continue

            filter_msg = String()
            filter_msg.data = payload_str
            self.filtered_pub.publish(filter_msg)
            self.get_logger().info(f"Received: {payload_str}")


def main(args=None):
    rclpy.init(args=args)
    node = XbeeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

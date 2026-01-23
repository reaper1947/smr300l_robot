#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import serial
import time

class SerialData:
    def __init__(self, port, baudrate=115200, interval=0.1):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        self.interval = interval
        self.sequence = 0

    def compute_checksum(self, data: str) -> int:
        checksum = 0
        for ch in data:
            checksum ^= ord(ch)
        return checksum

    def create_message(self):
        payload = f"DATA{self.sequence}"
        checksum = self.compute_checksum(payload)
        return f"{payload}|{checksum}\n"

    def send_once(self):
        message = self.create_message()
        self.ser.write(message.encode('ascii'))
        self.sequence += 1
        return message.strip()

    def read_response(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            if line:
                return line
        return None

    def close(self):
        if self.ser.is_open:
            self.ser.close()

class SerialSenderNode(Node):
    def __init__(self):
        super().__init__('xiao_node')

        # Serial port settings
        port = '/dev/xiao'
        baudrate = 115200
        self.timeout_sec = 1.0

        try:
            self.sender = SerialData(port, baudrate)
            self.port_state = True
            self.get_logger().info(f"Serial port {port} connected")
        except serial.SerialException as e:
            self.port_state = False
            self.get_logger().error(f"Cannot open port {port}: {e}")
            rclpy.shutdown()
            return

        # init
        self.last_recv_time = self.get_clock().now()

        # Publishers
        self.serial_pub = self.create_publisher(String, '/xiao_data', 10)
        self.heartbeat_pub = self.create_publisher(Bool, '/xiao_heartbeat', 10)

        # Timers
        self.send_timer = self.create_timer(0.1, self.send_data)
        self.heartbeat_timer = self.create_timer(0.1, self.send_heartbeat)

    def send_heartbeat(self):
        msg = Bool()
        if self.port_state == True:
            msg.data = True
        else:
            msg.data = False
        self.heartbeat_pub.publish(msg)

    def send_data(self):
        try:
            message = self.sender.send_once()
            # self.get_logger().info(f"Sent: {message}")
            self.serial_pub.publish(String(data=message))

            response = self.sender.read_response()
            if response:
                # self.get_logger().info(f"Received: {response}")
                self.serial_pub.publish(String(data=response))
                self.last_recv_time = self.get_clock().now()
            else:
                elapsed = self.get_clock().now() - self.last_recv_time
                if elapsed.nanoseconds > self.timeout_sec * 1e9:
                    self.get_logger().error("No response from XIAO — shutting down node.")
                    rclpy.shutdown()

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            self.port_state = False
            rclpy.shutdown()

    def destroy_node(self):
        self.get_logger().info("Shutting down node")
        self.sender.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

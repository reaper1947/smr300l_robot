#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.get_logger().info("Serial port opened")

        # Subscribing to control_command topic
        self.subscription = self.create_subscription(
            String,
            'control_command',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f"Received command: {command}")
        ## ros2 topic pub /control_command std_msgs/msg/String "data: ''"
        if command == "bypass":
            data = bytes([0x01, 0x0F, 0x00, 0x00, 0x00, 0x04, 0x01, 0x03, 0x7E, 0x97])
            self.send_data(data)

        elif command == "read_do":
            data = bytes([0x01, 0x01, 0x00, 0x00, 0x00, 0x08, 0x3D, 0xCC])
            self.send_and_parse_do(data)

        elif command == "read_di":
            data = bytes([0x01, 0x02, 0x00, 0x00, 0x00, 0x08, 0x79, 0xCC])
            self.send_and_parse_di(data)

        elif command == "close_all_do":
            data = bytes([0x01, 0x0F, 0x00, 0x00, 0x00, 0x08, 0x01, 0x00])
            crc = self.crc16(data)
            self.send_data(data + crc)

        elif command == "open_all_do":
            # A4 → 10100100
            data = bytes([0x01, 0x0F, 0x00, 0x00, 0x00, 0x08, 0x01, 0xFF])
            crc = self.crc16(data)
            self.send_data(data + crc)

        elif command.startswith("on_do_"):
            try:
                index = int(command.split("_")[2])
                self.turn_on_do(index)
            except:
                self.get_logger().error("Invalid DO number")

        else:
            self.get_logger().warn("Unknown command received.")

    def send_data(self, data: bytes):
        self.ser.write(data)
        time.sleep(0.1)
        response = self.ser.read(8)
        self.get_logger().info(f"Response: {response.hex()}")

    def send_and_parse_do(self, data: bytes):
        self.ser.write(data)
        time.sleep(0.1)
        response = self.ser.read(8)
        self.get_logger().info(f"DO Response: {response.hex()}")

        status = self.parse_do_status(response)
        for do, state in status.items():
            self.get_logger().info(f"{do}: {'ON' if state else 'OFF'}")

    def send_and_parse_di(self, data: bytes):
        self.ser.write(data)
        time.sleep(0.1)
        response = self.ser.read(8)
        self.get_logger().info(f"DI Response: {response.hex()}")

        if len(response) < 4:
            self.get_logger().error("Invalid DI response")
            return

        di_byte = response[3]
        for i in range(8):
            self.get_logger().info(f"DI_{i+1:02d}: {'ON' if (di_byte >> i) & 1 else 'OFF'}")

    def turn_on_do(self, index: int):
        if index < 1 or index > 8:
            self.get_logger().error("DO index must be 1-8")
            return

        addr = index - 1
        address_bytes = addr.to_bytes(2, byteorder='big')
        cmd = bytes([0x01, 0x05]) + address_bytes + bytes([0xFF, 0x00])
        crc = self.crc16(cmd)
        self.ser.write(cmd + crc)
        self.get_logger().info(f"Sent: turn ON DO_{index:02d}")

    def parse_do_status(self, response: bytes) -> dict:
        if len(response) < 4:
            return {"error": "Invalid response"}
        do_byte = response[3]
        return {f'DO_{i+1:02d}': bool((do_byte >> i) & 1) for i in range(8)}

    def crc16(self, data: bytes) -> bytes:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder='little')

    def destroy(self):
        self.ser.close()
        self.get_logger().info("Serial port closed")


def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

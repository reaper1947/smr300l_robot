#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
import struct
from std_msgs.msg import Bool

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.ser = None
        while self.ser is None:
            try:
                self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
                self.get_logger().info("Serial port opened")
            except serial.SerialException as e:
                self.get_logger().error(f"Could not open /dev/ttyACM0: {e}. Retrying in 2 seconds...")
                time.sleep(2)

        # Set RY1 & RY2 ON Start
        data_set_ry12 = bytes([0x01, 0x0F, 0x00, 0x00, 0x00, 0x04, 0x01, 0x03])
        crc_ry12 = self.crc16(data_set_ry12)
        data = data_set_ry12 + crc_ry12
        self.ser.write(data)
        self.get_logger().info("Set RY1 & RY2 ON at startup")
        # Create publisher for each RY status
        self.publisher_ry_status = {
            f'RY{i+1}': self.create_publisher(Bool, f'/ry_status/RY{i+1}', 10)
            for i in range(4)
        }

        time.sleep(0.1)

        self.timer = self.create_timer(1.0, self.send_and_receive)

    def send_and_receive(self):
        #Function 0x02
        data_read_di = bytes([0x01, 0x02, 0x00, 0x00, 0x00, 0x08])  # Read 8 inputs (X1-X8)
        crc_di = self.crc16(data_read_di)
        message_di = data_read_di + crc_di

        #Function 0x01
        data_read_ry = bytes([0x01, 0x01, 0x00, 0x00, 0x00, 0x04])  # Read 4 coils (RY1-RY4)
        crc_ry = self.crc16(data_read_ry)
        message_ry = data_read_ry + crc_ry

        #Function 0x05
        data_single_ry_ = bytes([0x01, 0x05, 0x00, 0x00, 0xFF, 0x00]) # FF00 = 1, 0000 = 0
        crc_ry = self.crc16(data_single_ry_)
        message_ry = data_single_ry_ + crc_ry

        #Function 0x0F
        data_several_ry = bytes([0x01, 0x0F, 0x00, 0x00, 0x00, 0x04, 0x01, 0x03]) ## 0F = 4 coil on
        crc_ry = self.crc16(data_several_ry)
        message_ry = data_several_ry + crc_ry

        #Function 0x06 address setting
        data_setting_ad = bytes([0x00, 0x06, 0x00, 0x64, 0x00, 0x01]) # 0001-00FE / 1-256
        crc_ry = self.crc16(data_setting_ad)
        message_ry = data_setting_ad + crc_ry

        #Function 0x06 bitrate setting
        data_setting_ad = bytes([0x01, 0x06, 0x00, 0x65, 0x00, 0x02]) # 0001 = 4800, 0002 = 9600 Manual topic 5.7
        crc_ry = self.crc16(data_setting_ad)
        message_ry = data_setting_ad + crc_ry


        self.ser.write(message_di)
        # self.get_logger().info("Read DI command sent")

        time.sleep(0.2)

        self.ser.write(message_ry)
        # self.get_logger().info("Read RY command sent")

        time.sleep(0.2)

        # response_di = self.ser.read(6)  # Expecting 6 bytes for DI
        # self.get_logger().info("DI Response: " + response_di.hex())
        # if len(response_di) != 6:
        #     self.get_logger().error("Invalid DI response length")
        #     return
        # if not self.verify_crc(response_di):
        #     self.get_logger().error("DI CRC check failed")
        #     return

        # di_status = self.parse_bit_status(response_di, 8)
        # for i, state in enumerate(di_status):
        #     self.get_logger().info(f"X{i+1}: {'ON' if state else 'OFF'}")


        response_ry = self.ser.read(6)  # Expecting 6 bytes for RY
        self.get_logger().info("RY Response: " + response_ry.hex())
        if len(response_ry) != 6:
            self.get_logger().error("Invalid RY response length")
            return
        # if not self.verify_crc(response_ry):
        #     self.get_logger().error("RY CRC check failed")
        #     return

        ry_status = self.parse_ry_status(response_ry)
        for ry, state in ry_status.items():
            self.get_logger().info(f"{ry}: {'ON' if state else 'OFF'}")

            msg = Bool()
            msg.data = state
            self.publisher_ry_status[ry].publish(msg)

    def parse_bit_status(self, response: bytes, bit_count: int):
        byte_val = response[3]
        return [(byte_val >> i) & 0x01 for i in range(bit_count)]

    def parse_ry_status(self, response: bytes) -> dict:
        ry_byte = response[3]  # 4 bits for RY1-RY4
        ry_status = {}
        for i in range(4):
            bit_state = (ry_byte >> i) & 0x01
            ry_status[f'RY{i+1}'] = bool(bit_state)
        return ry_status

    def crc16(self, data: bytes) -> bytes:
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 0x0001):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)

    def verify_crc(self, data: bytes) -> bool:
        if len(data) < 3:
            return False
        expected_crc = data[-2:]
        calc_crc = self.crc16(data[:-2])
        return expected_crc == calc_crc

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
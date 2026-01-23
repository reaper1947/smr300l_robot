import rclpy
from rclpy.node import Node
import serial
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.get_logger().info("Serial port opened")

        self.timer = self.create_timer(1.0, self.send_and_receive)

    def send_and_receive(self):
        data_by_pass = bytes([0x01, 0x0F, 0x00, 0x00, 0x00, 0x04, 0x01 , 0x03, 0x7E, 0x97])
        data_read_DO = bytes([0x01, 0x01, 0x00, 0x00, 0x00, 0x08, 0x3D, 0xCC])
        data_read_DI = bytes([0x01, 0x02, 0x00, 0x00, 0x00, 0x08, 0x79, 0xCC])
        data_close_DO = bytes([0x01, 0x0F, 0x00, 0x00, 0x00, 0x08, 0x01, 0x00])
        data_open_DO = bytes([0x01, 0x0F, 0x00, 0x00, 0x00, 0x08, 0x01, 0x00])

        self.ser.write(data_read_DO)
        self.get_logger().info("Command sent")

        time.sleep(0.1)

        response = self.ser.read(8)
        self.get_logger().info("Response: " + response.hex())

        status = self.parse_do_status(response)
        for do, state in status.items():
            self.get_logger().info(f"{do}: {'ON' if state else 'OFF'}")

    def turn_on_do(self, index: int):
        if index < 1 or index > 8:
            self.get_logger().error("DO index must be 1-8")
            return

        # DO_01 → address 0x0000, DO_02 → 0x0001, ...
        addr = index - 1
        address_bytes = addr.to_bytes(2, byteorder='big')
        cmd = bytes([0x01, 0x05]) + address_bytes + bytes([0xFF, 0x00])
        crc = self.crc16(cmd)
        full_cmd = cmd + crc
        self.ser.write(full_cmd)
        self.get_logger().info(f"Sent: turn ON DO_{index:02d}")

    def parse_do_status(self, response: bytes) -> dict:
        if len(response) < 4:
            return {"error": "Invalid response"}

        do_byte = response[3]
        do_status = {}

        for i in range(8):
            bit_state = (do_byte >> i) & 0x01
            do_status[f'DO_{i+1:02d}'] = bool(bit_state)

        return do_status

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

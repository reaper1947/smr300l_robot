#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import String, Bool
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from next2_msgs.msg import SeerRobotIOStatus, SeerIO
from next2_msgs.msg import Diagnostics
import time

class DeviceNode(Node):
    def __init__(self):
        super().__init__('device_io')
        # Declare parameters
        self.declare_parameter('connected', ' ')
        # self.declare_parameter('mode', 'unknown')
        self.di_names = [f'DI{i}' for i in range(1, 9)]
        self.ry_names = [f'RY{i}' for i in range(1, 5)]
        for name in self.di_names:
            self.declare_parameter(name, False, ParameterDescriptor(description=f'Status of {name}'))
        for name in self.ry_names:
            self.declare_parameter(name, False, ParameterDescriptor(description=f'Status of {name}'))

        # Create client to query parameter values
        self.client = self.create_client(GetParameters, '/logic_param_node/get_parameters')

        # Publishers
        self.enble_pub = self.create_publisher(Bool, 'io_heartbeat', 10) # heartbeat
        self.emergency_pub = self.create_publisher(Bool, 'emergency_io', 10)
        self.robot_pub = self.create_publisher(Bool, 'master_on', 10)
        self.io_pub = self.create_publisher(SeerRobotIOStatus, 'status_io', 10)

        # Subscription for external relay control
        self.create_subscription(String, 'command_rqt', self.relay_status, 10)
        self.create_subscription(Diagnostics, '/diagnostics_system_ready', self.ready, 10)

        # Internal state
        self.status = 0x00
        self.button_status = False
        self.di_status = SeerRobotIOStatus()
        self.ry_status = SeerRobotIOStatus()
        self.msg_robot = Bool()
        self.isSystemReady = 1

        # Timers
        self.enable = self.create_timer(0.1, self.check_enable)
        self.data_timer = self.create_timer(0.1, self.data_callback)
        self.check_timer = self.create_timer(0.1, self.check_command_rqt)

        # Serial initialization
        try:
            # self.ser = serial.Serial(port='/dev/device_io', baudrate=9600, parity='N', stopbits=1, bytesize=8, timeout=1.0)
            self.ser = serial.Serial(port='/dev/io', baudrate=9600, parity='N', stopbits=1, bytesize=8, timeout=1.0)
            self.get_logger().info("Serial port opened successfully.")
            self.enable_modbus = True
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.enable_modbus = False
            rclpy.shutdown()
            return
    
    def check_enable(self):
        msg = Bool()
        if self.enable_modbus == True:
            msg.data = True
        else:
            msg.data = False
        self.enble_pub.publish(msg)
    
    def ready(self, msg: Diagnostics):
        self.isSystemReady = msg.system_ready
 
    def relay_status(self, msg):
        # self.status = 0x03 if msg.data.lower() == 'relay on' else 0x00
        relay = msg.data
        
        if self.isSystemReady == 2 or relay == 'relay off':
            self.status = 0x00
            # print('offff')
        if relay == 'relay on':
            self.status = 0x03
            # print('onnnnn')

    def modbus_crc(self, data: bytes) -> bytes:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        return struct.pack('<H', crc)

    def sent_digital(self):
        request = struct.pack('>B B H H', 0x01, 0x02, 0x0000, 0x0008)
        request += self.modbus_crc(request)
        self.ser.write(request)
        # self.get_logger().info(f"Digital request sent: {request.hex().upper()}")

        digital_byte = self.read_digital()
        if not digital_byte & (1 << 0):  # Emergency (bit 0)
            self.status = 0x00
            self.button_status = False
        elif digital_byte & (1 << 3):  # Charge mode
            self.status = 0x00
            self.button_status = False
        # else:
        #     self.status = 0x00
        #     self.button_status = False

        if self.isSystemReady == 0 and not digital_byte & (1 << 3):
            if (digital_byte & (1 << 0)) and ((digital_byte & (1 << 1)) or (digital_byte & (1 << 2))):  # Master ON
                self.status = 0x03
                self.button_status = True

        # Set parameters based on relay states
        param_updates = []
        self.di_status.io_inputs.clear()
        for i, name in enumerate(self.di_names):
            value = bool((digital_byte >> i) & 1)
            param_updates.append(Parameter(name, Parameter.Type.BOOL, value))
            di = SeerIO()
            di.id = i+1
            di.source = "Digital_io"
            di.des = "   " 
            di.status = int(value)
            di.valid = value 
            self.di_status.io_inputs.append(di)
        self.set_parameters(param_updates)

        msg_emergency = Bool()
        msg_emergency.data = bool(digital_byte & (1 << 0))
        self.emergency_pub.publish(msg_emergency)

    def read_digital(self):
        response = self.ser.read(6)
        # self.get_logger().info(f"Digital response: {response.hex().upper()}")
        if len(response) != 6:
            self.get_logger().warn(f"Incomplete digital input response. Got {len(response)} bytes.")
            return None
        return response[3]

    def sent_relay(self):
        frame = struct.pack('>B B H H B B', 0x01, 0x0F, 0x0000, 0x0004, 0x01, self.status)
        full_frame = frame + self.modbus_crc(frame)
        self.ser.write(full_frame)
        # self.get_logger().info(f"Relay frame sent: {full_frame.hex().upper()}")

        response = self.ser.read(8)
        # if len(response) == 8:
        #     self.get_logger().info(f"Relay response: {response.hex().upper()}")
        # else:
        #     self.get_logger().warn(f"Incomplete relay response. Got {len(response)} bytes.")

        relay_byte = self.read_relay()
        # if relay_byte & (1 << 0) and relay_byte & (1 << 1):
        #     self.msg_robot.data = True
        # else:
        #     self.msg_robot.data = False

        # self.robot_pub.publish(self.msg_robot)

        param_updates = []
        self.ry_status.io_outputs.clear()
        for i, name in enumerate(self.ry_names):
            value = bool((relay_byte >> i) & 1)
            param_updates.append(Parameter(name, Parameter.Type.BOOL, value))
            ry = SeerIO()
            ry.id = i+1
            ry.source = "Relay_io"
            ry.des = " " 
            ry.status = int(value)
            ry.valid = value 
            self.ry_status.io_outputs.append(ry)
        self.set_parameters(param_updates)

    def read_relay(self):
        request = struct.pack('>B B H H', 0x01, 0x01, 0x0000, 0x0004)
        request += self.modbus_crc(request)
        self.ser.write(request)

        response = self.ser.read(6)
        if len(response) != 6:
            self.get_logger().warn(f"Incomplete relay state response. Got {len(response)} bytes.")
            return None
        return response[3]

    def data_callback(self):
        self.sent_digital()
        self.sent_relay()

        io_status = SeerRobotIOStatus()
        io_status.io_inputs = self.di_status.io_inputs
        io_status.io_outputs = self.ry_status.io_outputs
        self.io_pub.publish(io_status)

    def reset_relay(self):
        frame = struct.pack('>B B H H B B', 0x01, 0x0F, 0x0000, 0x0004, 0x01, 0x00)
        full_frame = frame + self.modbus_crc(frame)
        self.ser.write(full_frame)
        # self.get_logger().info(f"Reset relay sent: {full_frame.hex().upper()}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.reset_relay()
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

    def check_command_rqt(self):
        if self.enable_modbus == True:
            self.set_parameters([Parameter('connected', Parameter.Type.STRING, 'connected')])
            #get data from rqt
            request = GetParameters.Request()
            request.names = ['io_rqt']
            future = self.client.call_async(request)
            future.add_done_callback(self.handle_service_response)
        else:
            self.set_parameters([Parameter('connected', Parameter.Type.STRING, 'disconnected')])

    def handle_service_response(self, future):
        try:
            response = future.result()
            command_rqt = response.values[0].bool_value
            command = command_rqt or self.button_status

            # self.set_parameters([
            #     Parameter('mode', Parameter.Type.STRING, 'io readyyyyy' if command else 'unknown')
            # ])
            
        except Exception as e:
            self.get_logger().warn(f"Failed to handle service response: {e}")
        
def main(args=None):
    rclpy.init(args=args)
    node = DeviceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

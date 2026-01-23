#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Int64, Bool, String, Int32MultiArray, Int8MultiArray
from std_srvs.srv import SetBool
from next2_msgs.msg import RobotMode
from next2_msgs.srv import SetIOs
# from dynamic_reconfigure.server import Server
# from matrix_io_management.cfg import IOManagementConfig
from rcl_interfaces.srv import SetParameters

import math
from enum import Enum, auto

class ManagementIO(Node):
    class PublishMode(Enum):
        NOT_PUBLISH = auto()
        INACTIVE = auto()
        CHANGE = auto()
        RISING = auto()
        FALLING = auto()
        ACTIVE = auto()

    class ActionMode(Enum):
        NONE = auto()
        STOP_MOVEMENT = auto()
        STOP_ALL = auto()
        PAUSE = auto()
        RESUME = auto()
        EMERGENCY_CASE = auto()
        DOCKING = auto()

    class ISRdata:
        def __init__(self):
            self.input_address = -1
            self.current_state = 0
            self.last_state = 0
            self.mode = ManagementIO.PublishMode.NOT_PUBLISH
            self.data = ""
            self.action = ManagementIO.ActionMode.NONE

    def __init__(self):
        super().__init__('matrix_management_io')

        self.counter = 0

        # Publishers
        self.pub_raw_read = self.create_publisher(String, '/raw_read', 10)
        self.pub_bumper_state = self.create_publisher(Bool, '/matrix_io_management/bumper', 10)
        self.pub_usr_pin_x = self.create_publisher(Int8MultiArray, '/matrix_io_management/user_cfg_state/input', 10)
        self.pub_usr_pin_y = self.create_publisher(Int8MultiArray, '/matrix_io_management/user_cfg_state/output', 10)

        # Subscribers
        self.input_sub = self.create_subscription(
            Int32MultiArray,
            '/matrix_io/input',
            self.cbMCUInput,
            10)

        self.output_sub = self.create_subscription(
            Int32MultiArray,
            '/matrix_io/output',
            self.cbMCUOutput,
            10)

        self.robotmode_sub = self.create_subscription(
            RobotMode,
            '/matrix_mode_controller/mode',
            self.cbRobotMode,
            10)

        # Service Clients
        self.set_mode_sc = self.create_client(SetParameters, '/matrix_mode_controller/set_parameters')
        self.set_io_sc = self.create_client(SetIOs, 'matrix_io/service')

        # Initialize ISRdata objects
        self.x1 = self.ISRdata()
        self.x2 = self.ISRdata()
        self.x3 = self.ISRdata()
        self.x4 = self.ISRdata()
        self.x5 = self.ISRdata()
        self.x6 = self.ISRdata()
        self.x7 = self.ISRdata()
        self.x8 = self.ISRdata()
        self.x9 = self.ISRdata()
        self.x10 = self.ISRdata()
        self.xLimitSw = self.ISRdata()
        self.xBumper = self.ISRdata()
        self.xGD_button = self.ISRdata()
        self.xRD_button = self.ISRdata()
        self.xOG_button = self.ISRdata()

        # Previous states
        self.x1_prev = self.ISRdata()
        self.x2_prev = self.ISRdata()
        self.x3_prev = self.ISRdata()
        self.x4_prev = self.ISRdata()
        self.x5_prev = self.ISRdata()
        self.x6_prev = self.ISRdata()
        self.x7_prev = self.ISRdata()
        self.x8_prev = self.ISRdata()
        self.x9_prev = self.ISRdata()
        self.x10_prev = self.ISRdata()

        # Default states
        self.x1_de = self.ISRdata()
        self.x2_de = self.ISRdata()
        self.x3_de = self.ISRdata()
        self.x4_de = self.ISRdata()
        self.x5_de = self.ISRdata()
        self.x6_de = self.ISRdata()
        self.x7_de = self.ISRdata()
        self.x8_de = self.ISRdata()
        self.x9_de = self.ISRdata()
        self.x10_de = self.ISRdata()

        # Configuration parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('use_input_botton_state', False),
                ('use_input_bumper_state', False),
                ('use_STX_ETX', True),
                ('use_newline', False),
                ('active_state', 0),
                ('inactive_state', 1),
                ('hex_header', 0x02),
                ('hex_footer', 0x03),
                ('str_header', ''),
                ('str_footer', ''),
                ('x1_input_address', 10),
                ('x1_mode', self.PublishMode.RISING.value),
                ('x1_data', 'x1'),
                ('x1_action', self.ActionMode.NONE.value),
                # ... (add all other parameters similarly)
            ])

        # Initialize parameters
        self.use_input_botton_state = self.get_parameter('use_input_botton_state').value
        self.use_input_bumper_state = self.get_parameter('use_input_bumper_state').value
        self.use_STX_ETX = self.get_parameter('use_STX_ETX').value
        self.use_newline = self.get_parameter('use_newline').value
        self.active_state = self.get_parameter('active_state').value
        self.inactive_state = self.get_parameter('inactive_state').value
        self.hex_header = self.get_parameter('hex_header').value
        self.hex_footer = self.get_parameter('hex_footer').value
        self.str_header = self.get_parameter('str_header').value
        self.str_footer = self.get_parameter('str_footer').value

        # Initialize ISRdata objects with parameters
        self.x1.input_address = self.get_parameter('x1_input_address').value
        self.x1.mode = self.PublishMode(self.get_parameter('x1_mode').value)
        self.x1.data = self.get_parameter('x1_data').value
        self.x1.action = self.ActionMode(self.get_parameter('x1_action').value)

        # ... (initialize all other ISRdata objects similarly)

        # Copy to default and previous states
        self.use_STX_ETX_de = self.use_STX_ETX
        self.use_newline_de = self.use_newline

        # Timer for main loop
        self.timer = self.create_timer(0.1, self.main_loop)  # 10Hz

        self.get_logger().info("[matrix_io_management]: Ready!!")

    def main_loop(self):
        if self.wait_data_comeup and not self.init_dy_default:
            self.get_logger().info("Wait data comeup")
            # TODO: Implement dynamic reconfigure callback setup
            self.init_dy_default = True

    def cbRobotMode(self, msg):
        self.current_robotmode = msg.robot_mode

    def cbMCUInput(self, msg):
        self.x1.current_state = msg.data[self.x1.input_address]
        # ... (set current_state for all inputs)

        self.attachInterrupt(self.x1)
        # ... (call attachInterrupt for all inputs)

        self.x1.last_state = msg.data[self.x1.input_address]
        # ... (set last_state for all inputs)

        msg_pub = Int8MultiArray()
        msg_pub.data = [msg.data[10], msg.data[11], msg.data[12], msg.data[13]]
        self.pub_usr_pin_x.publish(msg_pub)

    def cbMCUOutput(self, msg):
        self.output_state = msg
        self.output = [msg.data[0], msg.data[1], msg.data[2], msg.data[3]]

        msg_pub = Int8MultiArray()
        msg_pub.data = [msg.data[10], msg.data[11], msg.data[12], msg.data[13]]
        self.pub_usr_pin_y.publish(msg_pub)

        self.wait_data_comeup = True

    def attachInterrupt(self, x):
        if x.input_address != -1:
            if x.mode == self.PublishMode.CHANGE:
                self.rise(x)
                self.fall(x)
            elif x.mode == self.PublishMode.RISING:
                self.rise(x)
            elif x.mode == self.PublishMode.FALLING:
                self.fall(x)
            elif x.mode == self.PublishMode.INACTIVE:
                self.inactive(x)
            elif x.mode == self.PublishMode.ACTIVE:
                self.active(x)
            elif x.mode == self.PublishMode.NOT_PUBLISH:
                pass

        self.pub_bumper(x)

    def rise(self, x):
        if x.current_state != x.last_state and x.current_state == self.active_state:
            self.get_logger().info("Rise Detect")
            self.set_robot_mode(x.action)
            self.pub_data(x.data)

    def fall(self, x):
        if x.current_state != x.last_state and x.current_state == self.inactive_state:
            self.get_logger().info("Fall Detect")
            self.set_robot_mode(x.action)
            self.pub_data(x.data)

    def inactive(self, x):
        if x.current_state == self.inactive_state:
            self.set_robot_mode(x.action)
            self.pub_data(x.data)

    def active(self, x):
        if x.current_state == self.active_state:
            self.set_robot_mode(x.action)
            self.pub_data(x.data)

    def pub_bumper(self, x):
        msg = Bool()
        if self.use_input_bumper_state and "Bumper" in x.data:
            msg.data = x.current_state == self.active_state
        else:
            msg.data = False
        self.pub_bumper_state.publish(msg)

    def pub_data(self, data):
        msg = String()
        STX = chr(0x02)
        ETX = chr(0x03)
        NL = '\n'

        if self.use_STX_ETX and self.use_newline:
            msg.data = STX + self.str_header + data + self.str_footer + ETX + NL
        elif self.use_STX_ETX and not self.use_newline:
            msg.data = STX + self.str_header + data + self.str_footer + ETX
        elif not self.use_STX_ETX and self.use_newline:
            msg.data = self.str_header + data + self.str_footer + NL
        else:
            msg.data = self.str_header + data + self.str_footer

        self.pub_raw_read.publish(msg)

    def set_robot_mode(self, mode):
        req = SetParameters.Request()

        if mode == self.ActionMode.STOP_MOVEMENT:
            param = Parameter()
            param.name = "stop_movement"
            param.value = True
            req.parameters.append(param)
        elif mode == self.ActionMode.STOP_ALL:
            param = Parameter()
            param.name = "stop_all"
            param.value = True
            req.parameters.append(param)
        elif mode == self.ActionMode.PAUSE:
            param = Parameter()
            param.name = "robot_mode"
            param.value = RobotMode.PAUSE
            req.parameters.append(param)
        elif mode == self.ActionMode.RESUME:
            param = Parameter()
            param.name = "robot_mode"
            param.value = 0  # clear all state
            req.parameters.append(param)
        elif mode == self.ActionMode.EMERGENCY_CASE:
            param = Parameter()
            param.name = "robot_mode"
            param.value = RobotMode.EMERGENCY_CASE_ACTIVE
            req.parameters.append(param)
        elif mode == self.ActionMode.DOCKING:
            param = Parameter()
            param.name = "robot_mode"
            param.value = RobotMode.DOCKING_MODE_ON
            req.parameters.append(param)

        if any(p.name == "robot_mode" for p in req.parameters):
            if self.current_robotmode != next(p.value for p in req.parameters if p.name == "robot_mode"):
                future = self.set_mode_sc.call_async(req)
        else:
            future = self.set_mode_sc.call_async(req)

        self.get_logger().info("set_mode success")

def main(args=None):
    rclpy.init(args=args)
    management_io = ManagementIO()
    rclpy.spin(management_io)
    management_io.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
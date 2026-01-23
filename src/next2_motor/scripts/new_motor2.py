#!/usr/bin/env python3

import os
os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

import can
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from next2_msgs.msg import RobotMode
import json
import sys
import time
import struct
import math
import threading
from rcl_interfaces.msg import SetParametersResult

class Ros2MotorNode(Node):
    def __init__(self):
        super().__init__('daniel_motor_node_ros2')

        self.lock = threading.Lock()
        self.logger = self.get_logger()

        # Parameters
        package_share_directory = os.path.join(
            os.getenv("HOME"), 'next_ros2/src/dan_ros_motor/config')
        self.json_file = os.path.join(package_share_directory, 'motor_param.json')
        self.param_data = self.load_from_json(self.json_file)
        for key, value in self.param_data.items():
            self.declare_parameter(key, value)
            setattr(self, key, self.get_parameter(key).value)

        # Bus Init
        self.bus = can.ThreadSafeBus(interface='socketcan', channel=self.can_dev, bitrate=self.baudrate)

        # CAN IDs
        self.bus_id = {
            "MOTOR_R": 0x181,
            "MOTOR_L": 0x182,
            "HEARTBEAT_R": 0x701,
            "HEARTBEAT_L": 0x702
        }

        # Publishers/Subscribers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.mode_sub = self.create_subscription(RobotMode, '/robot_mode', self.mode_callback, 10)
        self.masteron_sub = self.create_subscription(Bool, '/allowed_master_on', self.masteron_callback, 10)

        self.add_on_set_parameters_callback(self.param_reconfig)

        # Init vars
        self.v_left = 0.0
        self.v_right = 0.0
        self.robot_mode = 2
        self.current_mode = 'Auto'
        self.last_btn_state = True
        self.starting = True
        self.current_state = 'init'
        self.flow_states = {
            'init': self.send_start_msg,
            'alarm_check': self.motor_alarm_check,
            'auto_mode_check': self.auto_mode_check,
            'robot_mode_check': self.check_robot_mode,
            'enable_check': self.check_motor_enable,
            'spinning': self.spin_motor,
            'emergency_state': self.emergency_press,
            'recovery_state': self.emergency_release
        }

        # Timer
        self.create_timer(0.01, self.flow_state_machine)
        self.create_timer(1, self.check_heartbeat)



    def load_from_json(self, file):
        if not os.path.exists(file):
            self.logger.error('No parameter file found')
            return {}
        with open(file, 'r') as f:
            return json.load(f)

    def param_reconfig(self, params):
        with open(self.json_file, 'r') as f:
            exist_param = json.load(f)
        for param in params:
            data = {param.name: param.value}
            setattr(self, param.name, param.value)
            exist_param.update(data)
        with open(self.json_file, 'w') as f:
            json.dump(exist_param, f, indent=2)
        return SetParametersResult(successful=True)

    def cmd_callback(self, msg):
        self.v_left = msg.linear.x - (self.wheel_seperate_distance / 2.0) * msg.angular.z
        self.v_right = msg.linear.x + (self.wheel_seperate_distance / 2.0) * msg.angular.z

    def mode_callback(self, msg):
        self.robot_mode = msg.robot_mode

    def masteron_callback(self, msg):
        if self.starting and msg.data:
            self.starting = False
        elif msg.data != self.last_btn_state:
            self.current_state = 'emergency_state' if not msg.data else 'recovery_state'
            self.last_btn_state = msg.data

    def motor_transition(self, motor_id, transition):
        transitions = {
            'Shutdown': [0x2b, 0x40, 0x60, 0x00, 0x06],
            'Switchon': [0x2b, 0x40, 0x60, 0x00, 0x07],
            'Enable Operation': [0x2b, 0x40, 0x60, 0x00, 0x0f],
            'DIN1_unmap': [0x2f, 0x10, 0x20, 0x03, 0x00],
            'Enable Heartbeat': [0x2b, 0x17, 0x10, 0x00, 0x0a],
            'Enable CAN': [0x01, 0x00],
            'Set Profile Speed Mode': [0x2f, 0x20, 0x20, 0x0e, 0x03]
        }
        if transition in transitions:
            data = transitions[transition] + [0x00] * (8 - len(transitions[transition]))
            arb_id = 0x000 if transition == 'Enable CAN' else 0x600 | int(motor_id)
            self.bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=False))
        else:
            self.logger.error(f"Unknown transition: {transition}")

    def read_id(self, ids):
        start = time.monotonic()
        while time.monotonic() - start < 1:
            msg = self.bus.recv(timeout=0.1)
            if msg and msg.arbitration_id in ids:
                return msg
        return None

    def check_heartbeat(self):
        try:
            pulse = self.read_id([0x701, 0x702])
            self.is_heartbeat = pulse and pulse.data[0] == 0x05
        except:
            self.is_heartbeat = False

    def emergency_press(self):
        for mid in [self.motor_id_right, self.motor_id_left]:
            self.motor_transition(mid, 'Shutdown')
        self.v_left = 0.0
        self.v_right = 0.0

    def emergency_release(self):
        for mid in [self.motor_id_right, self.motor_id_left]:
            self.motor_transition(mid, 'Switchon')
            self.motor_transition(mid, 'Enable Operation')
        self.current_state = 'alarm_check'

    def send_start_msg(self):
        self.motor_transition("0", 'Enable CAN')
        for mid in [self.motor_id_right, self.motor_id_left]:
            self.motor_transition(mid, 'DIN1_unmap')
            self.motor_transition(mid, 'Enable Heartbeat')
            self.motor_transition(mid, 'Shutdown')
            self.motor_transition(mid, 'Switchon')
            self.motor_transition(mid, 'Enable Operation')
            self.motor_transition(mid, 'Set Profile Speed Mode')
        self.current_state = 'alarm_check'

    def motor_alarm_check(self):
        r = self.read_id([0x181])
        l = self.read_id([0x182])
        if r and l:
            self.current_state = 'auto_mode_check'
        else:
            self.logger.error("Motor alarm or CAN issue")
            self.disable_motor()

    def disable_motor(self):
        for mid in [self.motor_id_right, self.motor_id_left]:
            self.motor_transition(mid, 'Shutdown')
        self.current_state = 'init'

    def auto_mode_check(self):
        if self.current_mode != 'Auto':
            self.logger.error(f"Current mode: {self.current_mode}, not Auto")
            self.disable_motor()
        else:
            self.current_state = 'robot_mode_check'

    def check_robot_mode(self):
        if self.robot_mode != 2:
            self.logger.error(f"Current robot mode: {self.robot_mode}, not 2")
            self.disable_motor()
        else:
            self.current_state = 'enable_check'

    def check_motor_enable(self):
        r = self.read_id([0x181])
        l = self.read_id([0x182])
        if r and l:
            if not (r.data[4] & 2) or not (l.data[4] & 2):
                self.logger.error("Motors not enabled")
                self.enable_motor()
            else:
                self.current_state = 'spinning'

    def enable_motor(self):
        for mid in [self.motor_id_right, self.motor_id_left]:
            self.motor_transition(mid, 'Switchon')
            self.motor_transition(mid, 'Enable Operation')

    def cmd_to_rpm(self, v):
        rpm = (v / (math.pi * self.wheel_diameter)) * 60 * self.gear_ratio
        return min(self.motor_max_spd, rpm)

    def rpm_to_data_frame(self, rpm):
        objects_DEC = (int(rpm) * 512 * 2500 * 4) / 1875
        vel_hex = struct.pack('<i', int(objects_DEC))
        return [0x23, 0xFF, 0x60, 0x00] + list(vel_hex)

    def spin_motor(self):
        rpm_l = self.cmd_to_rpm(self.v_left)
        rpm_r = self.cmd_to_rpm(self.v_right)
        msg_l = can.Message(arbitration_id=0x600 | self.motor_id_left,
                            data=self.rpm_to_data_frame(-rpm_l), is_extended_id=False)
        msg_r = can.Message(arbitration_id=0x600 | self.motor_id_right,
                            data=self.rpm_to_data_frame(rpm_r), is_extended_id=False)
        self.bus.send(msg_r)
        self.bus.send(msg_l)
        self.current_state = 'alarm_check'

    def flow_state_machine(self):
        if not self.starting and getattr(self, 'is_heartbeat', False):
            handler = self.flow_states.get(self.current_state, None)
            if handler:
                handler()


def main(args=None):
    rclpy.init(args=args)
    motor_node = Ros2MotorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(motor_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        motor_node.get_logger().info("Shutting down...")
    finally:
        motor_node.bus.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

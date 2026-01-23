#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64, String, Bool
from std_srvs.srv import SetBool
from next2_msgs.msg import RobotMode
import can
import struct
import json
from struct import *
import time
import math
from rcl_interfaces.msg import ParameterDescriptor
from enum import Enum


class DeviceStateCanopen:
    STATE_FAULT = 0x01
    STATE_OPERATION_ENABLED = 0x02
    STATE_SWITCH_ON_DISABLED = 0x03
    STATE_UNKNOW = 0xFF


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Initializing parameters with defaults or from parameter server
        self.declare_parameter("device_name", "kinco")
        self.declare_parameter("wheels_x_distance", 0.0)
        self.declare_parameter("wheels_y_distance", 0.49)
        self.declare_parameter("wheel_diameter", 0.2)
        self.declare_parameter("max_motor_rpm", 3000.0)
        self.declare_parameter("gear_ratio", 20.00)
        self.declare_parameter("total_wheels", 2.0)
        self.declare_parameter("can_bitrate", 500000)
        self.declare_parameter("motor_1_id", 1)
        self.declare_parameter("motor_2_id", 2)
        self.device_name = self.get_parameter("device_name").value
        self.wheels_x_distance_ = self.get_parameter("wheels_x_distance").value
        self.wheels_y_distance_ = self.get_parameter("wheels_y_distance").value
        self.wheel_diameter = self.get_parameter("wheel_diameter").value
        self.max_motor_rpm = self.get_parameter("max_motor_rpm").value
        self.gear_ratio = self.get_parameter("gear_ratio").value
        self.total_wheels_ = self.get_parameter("total_wheels").value
        self.can_bitrate = self.get_parameter("can_bitrate").value
        self.motor_1_id = self.get_parameter("motor_1_id").value
        self.motor_2_id = self.get_parameter("motor_2_id").value

        self.wheel_circumference_ = math.pi * self.wheel_diameter
        self.can_interface = 'can0'

        self.STW_181 = {
            'ReadytoSwitchON': 0,
            'SwitchedON': 0,
            'OperationEnabled': 0,
            'Fault': 0,
            'VoltageEnabled': 0,
            'QuickStop': 0,
            'SwitchONDisabled': 0,
            'Warning': 0
        }
        self.STW_182 = {
            'ReadytoSwitchON': 0,
            'SwitchedON': 0,
            'OperationEnabled': 0,
            'Fault': 0,
            'VoltageEnabled': 0,
            'QuickStop': 0,
            'SwitchONDisabled': 0,
            'Warning': 0
        }

        # Motor configuration
        self.motor_1_id = 1
        self.motor_2_id = 2
        self.motor_3_id = 3
        self.test = True

        # State Machine
        self.COMMAND_RATE = 20
        self.prev_rpm = 0
        self.timeout_state = 0
        self.speed_e = 0
        self._isMotor_1_OperationEn = False
        self._isMotor_2_OperationEn = False
        self._isFault_Detected = False
        self._isAllowed_Master_ON = False
        self._isAuto_mode = True
        self._isInitial_Manual = False
        self._isSkip_Allowed_Master_ON = False
        self._isManuaMode = False
        self.current_robotmode = RobotMode()
        self.current_motor_state = "INIT"
        # self.current_robotmode = 2

        self.ControlState = Enum('ControlState', ['INIT', 'MANUAL_MODE', 'ROBOT_SYSTEM_CHECK', 'MOTOR_STATE_CHECK',
                                                  'INIT_MOTOR', 'WAIT_STATUSWORD_ENABLED', 'BASE_CONTROL',
                                                  'FINAL', 'FAULT_CHECKING', 'FAULT_DETECTED', 'CLEAR_FAULT',
                                                  'ERROR', 'PAUSE'])

        self.ControlState_seq = self.ControlState.INIT.value

        # Add robot_mode_map for readable mode names
        self.robot_mode_map = {
            1: "IDLE",
            2: "START_MOTOR",
            3: "SHUTDOWN_MOTOR",
            4: "SHUTDOWN_ROBOT",
            5: "EMERGENCY",
            6: "FUCN_1",
            7: "DOWN_STAIRS",
            8: "TABLET_LOSS_COMMU",
            9: "UVC_ON",
            10: "UVC_OFF",
            11: "READY_TO_START",
            12: "DOCKING_MODE_ON",
            13: "DOCKING_MODE_OFF",
            14: "ERROR_DEVICE",
            15: "CHARGER_ON",
            16: "CHARGER_OFF",
            17: "RESET",
            18: "EMERGENCY_CASE_ACTIVE",
            19: "EMERGENCY_CHARGE",
            20: "MANUAL_DOCKING",
            21: "ROBOT_OPERATION_BEGIN",
            22: "ROBOT_OPERATION_FINISH",
            23: "PAUSE",
            24: "RESUME",
            25: "WAIT_FOR_CONNECT_MASTER",
            26: "MAPPING",
            27: "GO_TO_DOCKING",
            28: "FULLCHARGE",
            29: "LOW_BATTERY",
            30: "BOOTING"
        }

        # Initialize CAN bus
        filters = [
            {"can_id": 0x181, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x182, "can_mask": 0x7FF, "extended": False},
        ]
        self.can_bus = can.interface.Bus(interface='socketcan', channel='can0', can_filters=filters)
        self.bus = can.interface.Bus(interface='socketcan', channel='can0')

        # ROS2 Publishers and Subscribers
        self.statusword_pub = self.create_publisher(String, "statusword", 10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.mode_sub = self.create_subscription(
            RobotMode,
            '/robot_mode',
            self.mode_callback, 10)

        self.masteron_sub = self.create_subscription(
            Bool,
            '/allowed_master_on',
            self.allowed_callback, 10) # allowed_master_on

        # Initialize motor
        self.NMT_service(0x00, "Reset Node")
        self.init_motor()
        while not self.test == False:
            print(self.current_robotmode)
            self.mode_callback

        # Create timer for control loop
        self.timer = self.create_timer(0.02, self.control_loop)  # 50Hz


    def mode_callback(self, msg):
        self.current_robotmode = msg
        mode_num = msg.robot_mode
        mode_str = self.robot_mode_map.get(mode_num, f"UNKNOWN({mode_num})")
        self.get_logger().info(f"[ModeCallback] Mode: {mode_num} ({mode_str})")


    def allowed_callback(self, msg):
        self._isAllowed_Master_ON = msg.data

    def control_loop(self):
        self.ControlLoop_State()
        self.DiagnosticsStatusWordnPublish()
    def publish_statusword(self):
        statusword_message = {
            "STW_181": self.STW_181,
            "STW_182": self.STW_182
        }
        msg = String()
        msg.data = json.dumps(statusword_message)
        self.statusword_pub.publish(msg)

    def ControlLoop_State(self):
        if self.current_robotmode.robot_mode == 2:
            self.NMT_service(0x00, "Switch to the <Operational> state")
        elif not self.current_robotmode.robot_mode == 2:
            data_frame = [0x40, 0x6c, 0x60, 0x00, 0x02,
                          0x00, 0x00, 0x00]  # controlWord "Quick Stop"
            self.SendToCan(0x600 + self.motor_1_id, data_frame)
            time.sleep(0.1)
            self.SendToCan(0x600 + self.motor_2_id, data_frame)
            time.sleep(0.1)
            self.NMT_service(0x00, "Switch to the <Stoped> state")

    def stopBase(self, msg):
        pass

    def cmd_vel_callback(self, msg):
        try:
            if self.current_robotmode.robot_mode == 2:
                linear_velocity = msg.linear.x
                angular_velocity = msg.angular.z
            else:
                linear_velocity = 0
                angular_velocity = 0
        except BaseException:
            pass

        left_rpm, right_rpm = self.calculate_rpm(linear_velocity, angular_velocity)
        self.SDO_MotorSpin(self.motor_1_id, left_rpm)
        self.SDO_MotorSpin(self.motor_2_id, right_rpm)

    def calculate_rpm(self, linear_x, angular_z):
        # convert m/s to m/min
        linear_vel_x_mins = linear_x * 60
        linear_vel_y_mins = 0

        # convert rad/s to rad/min
        angular_vel_z_mins = -angular_z * 60

        tangential_vel = angular_vel_z_mins * \
            ((self.wheels_x_distance_ / 2) + (self.wheels_y_distance_ / 2))
        x_rpm = linear_vel_x_mins / self.wheel_circumference_
        y_rpm = linear_vel_y_mins / self.wheel_circumference_
        tan_rpm = tangential_vel / self.wheel_circumference_

        rpm_motor_left = x_rpm + y_rpm - tan_rpm
        rpm_motor_right = -(x_rpm + y_rpm + tan_rpm)

        rpm_motor_left *= self.gear_ratio
        rpm_motor_right *= self.gear_ratio

        rpm_motor_left = min(self.max_motor_rpm, max(-self.max_motor_rpm, rpm_motor_left))
        rpm_motor_right = min(self.max_motor_rpm, max(-self.max_motor_rpm, rpm_motor_right))

        value_motor_left = rpm_motor_left
        value_motor_right = rpm_motor_right
        return [value_motor_left, value_motor_right]

    def DiagnosticsStatusWordnPublish(self):
        timeout = self.get_clock().now() + rclpy.time.Duration(seconds=1.0)
        received_181 = False
        received_182 = False

        while not (received_181 and received_182):
            frame = self.bus.recv(timeout=0.1)
            if frame is None:
                self.get_logger().warn("Timeout waiting for CAN message", throttle_duration_sec=2)
                break

            if frame.arbitration_id == 0x181:
                sw_h_181 = bytearray([frame.data[4], frame.data[5]])
                sw_d_181 = unpack('<h', sw_h_181)
                sw_d_181 = sw_d_181[0]
                sw_b_181 = bin(sw_d_181)[2:].zfill(16)
                self.STW_181['ReadytoSwitchON'] = int(sw_b_181[-1])
                self.STW_181['SwitchedON'] = int(sw_b_181[-2])
                self.STW_181['OperationEnabled'] = int(sw_b_181[-3])
                self.STW_181['Fault'] = int(sw_b_181[-4])
                self.STW_181['VoltageEnabled'] = int(sw_b_181[-5])
                self.STW_181['QuickStop'] = int(sw_b_181[-6])
                self.STW_181['SwitchONDisabled'] = int(sw_b_181[-7])
                self.STW_181['Warning'] = int(sw_b_181[-8])
                received_181 = True

            if frame.arbitration_id == 0x182:
                sw_h_182 = bytearray([frame.data[4], frame.data[5]])
                sw_d_182 = unpack('<h', sw_h_182)
                sw_d_182 = sw_d_182[0]
                sw_b_182 = bin(sw_d_182)[2:].zfill(16)
                self.STW_182['ReadytoSwitchON'] = int(sw_b_182[-1])
                self.STW_182['SwitchedON'] = int(sw_b_182[-2])
                self.STW_182['OperationEnabled'] = int(sw_b_182[-3])
                self.STW_182['Fault'] = int(sw_b_182[-4])
                self.STW_182['VoltageEnabled'] = int(sw_b_182[-5])
                self.STW_182['QuickStop'] = int(sw_b_182[-6])
                self.STW_182['SwitchONDisabled'] = int(sw_b_182[-7])
                self.STW_182['Warning'] = int(sw_b_182[-8])
                received_182 = True

            # STATUS CIA402 0x181
            if self.STW_181['Fault']:
                self.current_motor_state = "Fault"
            elif self.STW_181['ReadytoSwitchON'] and self.STW_181['SwitchedON'] and self.STW_181['OperationEnabled']:
                self.current_motor_state = "Enabled Operation"
            elif self.STW_181['SwitchONDisabled']:
                self.current_motor_state = "Switech on disabled"
            else:
                self.current_motor_state = "UNKNOW"

            # STATUS CIA402 0x182
            if self.STW_182['Fault']:
                self.current_motor_state = "Fault"
            elif self.STW_182['ReadytoSwitchON'] and self.STW_182['SwitchedON'] and self.STW_182['OperationEnabled']:
                self.current_motor_state = "Enabled Operation"
            elif self.STW_182['SwitchONDisabled']:
                self.current_motor_state = "Switech on disabled"
            else:
                self.current_motor_state = "UNKNOW"

        self.publish_statusword()

    def SendToCan(self, arbitration_id, data_frame):
        try:
            msg = can.Message(arbitration_id=arbitration_id, data=data_frame, is_extended_id=False)
            self.can_bus.send(msg)
            return True
        except can.CanError as e:
            self.get_logger().error(f"CAN message sending failed: {e}")
            return False

    def StartNode(self):
        self.get_logger().info("***************************************************************************************")
        self.get_logger().info("**************************************DRIVE MOTOR**************************************")
        self.get_logger().info("***********************************KINCO MOTOR START***********************************")
        self.get_logger().info("***********************************KINCO MOTOR START***********************************")
        self.get_logger().info("***********************************KINCO MOTOR START***********************************")
        self.get_logger().info("***************************************************************************************")
        time.sleep(1)
        self.NMT_service(0x00, "Reset Node")
        time.sleep(0.1)
        self.NMT_service(0x00, "Switch to the <Operational> state")
        time.sleep(0.1)
        self.NMT_service(0x02, "Switch to the <Operational> state")
        time.sleep(0.1)

    def KillNode(self):
        self.get_logger().info("***************************************************************************************")
        self.get_logger().info("**************************************DRIVE MOTOR**************************************")
        self.get_logger().info("***********************************KINCO MOTOR STOP************************************")
        self.get_logger().info("***********************************KINCO MOTOR STOP************************************")
        self.get_logger().info("***********************************KINCO MOTOR STOP************************************")
        self.get_logger().info("***************************************************************************************")
        self.NMT_service(0x00, "Reset Node")

    def init_motor(self):
        success = False
        self.get_logger().info("Initializing motor...")
        if self.device_name == "kinco":
            try:
                commands = [
                    [0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00],
                ]
                for command in commands:
                    msg = can.Message(arbitration_id=0x601, data=command, is_extended_id=False)
                    msg = can.Message(arbitration_id=0x602, data=command, is_extended_id=False)
                    self.can_bus.send(msg)
                    time.sleep(0.1)
                self.get_logger().info("Motor initialized successfully!")
            except can.CanError as e:
                self.get_logger().error(f"CAN message sending failed during initialization: {e}")
        else:
            self.get_logger().warn("Unsupported device!")

    def SDOclearErrorDualMotor(self):
        if self.device_name == "kinco":
            self.SDO_clearError(self.dual_motor_id)
        else:
            self.SDO_clearError(self.motor_1_id)
            self.SDO_clearError(self.motor_2_id)

    def SDO_clearError(self, MotorID):
        success = False
        if self.device_name == "OrientalmotorBLVD_KRD":
            data_frame = [0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00]
        elif self.device_name == "kinco":
            data_frame = [0x2b, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00]
        success = self.SendToCan(0x600 + MotorID, data_frame)
        return success

    def MotorSwitched(self, MotorID, cmd):
        success = True
        data = 0x00
        if cmd == "Shutdown":
            data = 0x06
        elif cmd == "Switch ON":
            data = 0x07
        elif cmd == "Switch ON + Enable Operation":
            data = 0x0F
        elif cmd == "Disable Voltage":
            data = 0x00
        elif cmd == "Quick Stop":
            data = 0x02
        elif cmd == "Disable Operation":
            data = 0x03
        elif cmd == "Enable Operation":
            data = 0x0F
        elif cmd == "Fault Reset":
            data = 0x80
        else:
            success = False

        if success:
            data_frame = [0x2B, 0x40, 0x60, 0x00, data, 0x00, 0x00, 0x00]
            success = self.SendToCan(0x600 + MotorID, data_frame)
            if success:
                self.get_logger().info(
                    f"Success set Status Machine control commands Motor-ID {MotorID}, cmd {cmd}")
            else:
                self.get_logger().error(
                    f"Fail set Status Machine control commands Motor-ID {MotorID}, cmd {cmd}")
        else:
            self.get_logger().error(f"Not found Status Machine control commands cmd {cmd}")

        return success

    def SDO_MotorSpin(self, MotorID, target_vel):
        success = False
        objects_DEC = (int(target_vel) * 512 * 2500 * 4) / 1875
        vel_hex = struct.pack('<i', int(objects_DEC))
        data_frame = [
            vel_hex[0],
            vel_hex[1],
            vel_hex[2],
            vel_hex[3]]

        success = self.SendToCan(0x170 + MotorID, data_frame)
        return success

    def SDO_ModeofOperation(self, MotorID, mode):
        success = False
        data_frame = []
        if mode == "Profile Velocity Mode":
            data_frame = [0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00]
            success = True
        else:
            success = False

        if success:
            success = self.SendToCan(0x600 + MotorID, data_frame)
        return success

    def SDO_SendReqMotorActualSpeed(self, MotorID):
        if self.device_name == "kinco":
            data_frame = [0x40, 0x6c, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
        success = self.SendToCan(0x600 + MotorID, data_frame)
        return success

    def get_rpm(self):
        rpm1, rpm2 = 0, 0
        received_rpm1 = False
        received_rpm2 = False
        timeout = self.get_clock().now() + rclpy.time.Duration(seconds=1.0)

        while not (received_rpm1 and received_rpm2):
            frame = self.bus.recv(timeout=0.1)
            if frame is None:
                self.get_logger().warn("Timeout waiting for CAN message")
                break

            if frame.arbitration_id == 0x181:
                try:
                    m1 = unpack('<i', frame.data[0:4])[0]
                    rpm1 = (m1 * 1875 / (512 * 2500 * 4)) * self.velocity_unit_rpm
                    received_rpm1 = True
                    if -10 <= rpm1 <= 10:
                        rpm1 = 0
                        self.get_logger().info(f"rpm1: {int(rpm1)}", throttle_duration_sec=1)
                    else:
                        self.get_logger().info(f"rpm1: {int(rpm1)}", throttle_duration_sec=1)
                except Exception as e:
                    self.get_logger().error(f"Error parsing frame data: {e}")

            if frame.arbitration_id == 0x182:
                try:
                    m2 = unpack('<i', frame.data[0:4])[0]
                    rpm2 = (m2 * 1875 / (512 * 2500 * 4)) * self.velocity_unit_rpm
                    received_rpm2 = True
                    if -10 <= rpm2 <= 10:
                        rpm2 = 0
                        self.get_logger().info(f"rpm2: {int(rpm2)}", throttle_duration_sec=0.1)
                    else:
                        self.get_logger().info(f"rpm2: {int(rpm2)}", throttle_duration_sec=0.1)
                except Exception as e:
                    self.get_logger().error(f"Error parsing frame data: {e}")

            if self.get_clock().now() > timeout:
                self.get_logger().error("Failed to receive all RPM data within timeout")
                break

        return rpm1, rpm2

    def SDO_SendReqStatusWordDualMotor(self):
        if self.device_name == "kinco":
            self.SDO_SendReqStatusWord(self.motor_1_id)
            self.SDO_SendReqStatusWord(self.motor_2_id)

    def SDO_DisableOperationEnabled(self, MotorID):
        success = False
        if self.MotorSwitched(MotorID, "Quick Stop"):
            if self.MotorSwitched(MotorID, "Disable Voltage"):
                success = True
        return success

    def NMT_service(self, MotorID, cmd):
        success = True
        COB_ID = 0x00
        Byte0_cmd = 0x00
        Byte1_NodeID = MotorID
        if cmd == "Switch to the <Operational> state":
            Byte0_cmd = 0x01
        elif cmd == "Switch to the <Stoped> state":
            Byte0_cmd = 0x02
        elif cmd == "Switch to the <Pre-operational> state":
            Byte0_cmd = 0x80
        elif cmd == "Reset Node":
            Byte0_cmd = 0x81
        elif cmd == "Reset Communication":
            Byte0_cmd = 0x82
        else:
            success = False

        if success:
            data_frame = [Byte0_cmd, MotorID]
            success = self.SendToCan(COB_ID, data_frame)
        return success


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math

from std_msgs.msg import Int8, Bool, Int32MultiArray, String
from std_srvs.srv import SetBool
from next2_msgs.msg import Diagnostics, RobotMode
from next2_msgs.srv import ActionController, SetIOs
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration

MCU_SIGNAL_ACTIVE = 0

class MatrixModeController(Node):
    def __init__(self):
        super().__init__('matrix_mode_controller')

        self.counter = 0

        # Parameters
        self.declare_parameter('serial_no', 'DELI010020220001A')
        self.declare_parameter('model', 'delivery')
        self.declare_parameter('enable_emergency_charge_state', True)
        self.declare_parameter('emergency_charge_active_value', 0)
        self.declare_parameter('emergency_plugin_active_value', 0)
        self.declare_parameter('emergency_base_active_value', 0)
        self.declare_parameter('permissive_on_when_bumper_detect', True)

        self.serial_no = self.get_parameter('serial_no').value
        self.model = self.get_parameter('model').value
        self.enable_emergency_charge_state = self.get_parameter('enable_emergency_charge_state').value
        self.emergency_charge_active_value = self.get_parameter('emergency_charge_active_value').value
        self.emergency_plugin_active_value = self.get_parameter('emergency_plugin_active_value').value
        self.emergency_base_active_value = self.get_parameter('emergency_base_active_value').value
        self.permissive_on_when_bumper_detect = self.get_parameter('permissive_on_when_bumper_detect').value

        # Callback group for services to allow parallel execution
        self.cb_group = ReentrantCallbackGroup()

        # Publishers
        self.pub_mode = self.create_publisher(RobotMode, '/matrix_mode_controller/mode', 10)
        self.pub_allowed_master_on = self.create_publisher(Bool, '/matrix_mode_controller/allowed_master_on', 10)

        # Subscribers
        self.diagnostics_subscriber = self.create_subscription(
            Diagnostics, '/matrix_system/diagnostics/system_ready',
            self.cbDiag, 10)

        self.sub_emer_charge = self.create_subscription(
            Bool, '/matrix_system/diagnostics/emer_charge',
            self.cbEmerCharge, 10)

        self.sub_emer_state = self.create_subscription(
            Int8, '/matrix_io/emergency',
            self.cbEmerState, 10)

        self.sub_batt_state = self.create_subscription(
            BatteryState, 'battery_state',
            self.cbBatteryState, 10)

        self.sub_input = self.create_subscription(
            Int32MultiArray, '/matrix_io/input',
            self.cbInput, 10)

        self.sub_output = self.create_subscription(
            Int32MultiArray, 'matrix_io/output',
            self.cbOutput, 10)

        self.sub_emer_charge_state = self.create_subscription(
            Bool, '/matrix_system/diagnostics/emergency_charge',
            self.cbEmerChargeState, 10)

        self.sub_bumper = self.create_subscription(
            Bool, '/matrix_io_management/bumper',
            self.cbBumper, 10)

        self.sub_odom = self.create_subscription(
            Odometry, '/odom',
            self.cbOdom, 10)

        # Services
        self.docking_service = self.create_service(
            SetBool, '/matrix_mode_controller/docking_mode',
            self.cbDocking, callback_group=self.cb_group)

        self.pause_service = self.create_service(
            SetBool, '/matrix_mode_controller/pause_mode',
            self.cbPause, callback_group=self.cb_group)

        self.set_mode_service = self.create_service(
            ActionController, '/matrix_mode_controller/set_mode',
            self.cbSetMode, callback_group=self.cb_group)

        # Service Clients
        self.set_ios_cli = self.create_client(SetIOs, 'matrix_io/service')
        while not self.set_ios_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('matrix_io/service service not available, waiting again...')

        self.action_launch_cli = self.create_client(ActionController, 'matrix_launch_controller/action_controller')
        while not self.action_launch_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('matrix_launch_controller/action_controller service not available, waiting again...')

        self.action_launch2_cli = self.create_client(SetBool, 'matrix_launch_controller/cancel_action')
        while not self.action_launch2_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('matrix_launch_controller/cancel_action service not available, waiting again...')

        # Initialize variables
        self._isSystemReady = Diagnostics.SYSTEM_READY_WAIT_COMEUP
        self._isEmerCharge_mode = False
        self._isEmerState_mode = 1
        self._isDocking_mode = False
        self._isPause = False
        self._isErrorDevice_state = False
        self._isMCUMaster_on = False
        self._isMCUMaster_on_done = False
        self._isMCUEmergency = False
        self._isMCUEmergencyPlugin = False
        self._isMCUEmergency_charge = False
        self._isMCUComputer_ready = False
        self._isMCUShutdown_robot = False
        self._isMCUEnable_charger = False
        self._isMCUBumper = False
        self._isSOFTWAREEmergency_charge = False
        self._isInit_state = False
        self._isInit_motor = False
        self._isSystemReady_comeup = False
        self._isErrorDevice = False
        self._isEmergency_state = False
        self._isEmergency_state_prev = False
        self._isEmerCharge_state = False
        self._isEmergency_case_active = False
        self._isOperating = False
        self._isShutdown = False
        self._isCharging = False
        self._isOn = False
        self._isMasterOnSys = False
        self._isBumperPush = False
        self._isAnotherAutocharge = False
        self.emergency_charge_active_value = 0
        self.emergency_plugin_active_value = 0
        self.emergency_base_active_value = 0
        self.prev_robot_mode = 0
        self.before_robot_mode = 0
        self.twist = Twist()
        self.mode_msg = RobotMode()
        self.current_mode = RobotMode()
        self.allowed_master_on_msg = Bool()

        # Operation sequence state
        self.operation_sequence_ = self.operation_sequence.init_io

        self.get_logger().info("[MatrixModeController]: Initialization complete")

        # Create timer for main loop
        self.timer = self.create_timer(0.04, self.state_controller_sequence)  # ~25Hz

    class operation_sequence:
        init_io = 0
        check_sysready = 1
        check_button_state = 2
        check_emer_charge = 3
        pub_botton_state = 4
        control_state = 5

    def cbOdom(self, msg):
        self.twist = msg.twist.twist

    def cbBumper(self, msg):
        self._isBumperPush = msg.data

    def invertMCUState(self, signal, active_signal=MCU_SIGNAL_ACTIVE):
        if active_signal == -1:
            return False
        return signal == active_signal

    def cbEmerChargeState(self, msg):
        self._isSOFTWAREEmergency_charge = msg.data

    def cbInput(self, msg):
        self._isMCUEmergency = self.invertMCUState(msg.data[0], self.emergency_base_active_value)
        self._isMCUMaster_on = self.invertMCUState(msg.data[1])
        self._isMCUMaster_on_done = self.invertMCUState(msg.data[2])
        self._isMCUEmergency_charge = self.invertMCUState(msg.data[3], self.emergency_charge_active_value) if self.enable_emergency_charge_state else False
        self._isMCUEmergencyPlugin = self.invertMCUState(msg.data[4], self.emergency_plugin_active_value)

    def cbOutput(self, msg):
        self._isMCUComputer_ready = self.invertMCUState(msg.data[0], 1)
        self._isMCUShutdown_robot = self.invertMCUState(msg.data[1], 1)
        self._isMCUEnable_charger = self.invertMCUState(msg.data[3], 1)

    def cbBatteryState(self, msg):
        self._isCharging = msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING

    def cbEmerState(self, msg):
        self._isEmerState_mode = msg.data

    def cbDiag(self, msg):
        self._isSystemReady = msg.system_ready
        self._isSystemReady_comeup = True

    def cbEmerCharge(self, msg):
        self._isEmerCharge_mode = msg.data

    async def cbSetMode(self, request, response):
        if request.cmd == "NORMAL":
            if request.arg0 == "ACTIVE":
                self._isPause = False
                self._isDocking_mode = False
                self._isEmergency_case_active = False
                self._isOn = False
                response.success = True
                response.message = "[MatrixModeController]:Normal mode"
                self._isInit_state = False
            else:
                response.success = True
                response.message = "[MatrixModeController]:Normal mode disable"

        elif request.cmd == "STOP_MOVEMENT":
            srv = ActionController.Request()
            srv.cmd = "action_cancel"
            srv.action_name = "movement"
            try:
                future = self.action_launch_cli.call_async(srv)
                await future
                response.success = future.result().success
                response.message = future.result().message
            except Exception as e:
                response.success = False
                response.message = str(e)

        elif request.cmd == "STOP_ALL":
            srv1 = ActionController.Request()
            srv1.cmd = "action_cancel"
            srv1.action_name = "STOP_ALL"

            srv2 = SetBool.Request()
            srv2.data = True

            try:
                future1 = self.action_launch_cli.call_async(srv1)
                future2 = self.action_launch2_cli.call_async(srv2)

                await future1
                await future2

                response.success = future1.result().success and future2.result().success
                response.message = f"{future1.result().message}, {future2.result().message}"
                self.get_logger().warn("[MatrixModeController]:cancel")
            except Exception as e:
                response.success = False
                response.message = str(e)

        elif request.cmd == "PAUSE":
            if request.arg0 == "ACTIVE":
                self._isPause = True
                response.success = True
                response.message = "[MatrixModeController]:Pause enable"
            else:
                response.success = True
                response.message = "[MatrixModeController]:Pause disable"

        elif request.cmd == "RESUME":
            if request.arg0 == "ACTIVE":
                if self.permissive_on_when_bumper_detect or not self._isBumperPush:
                    self._isPause = False
                    self._isInit_state = False
                    response.success = True
                    self._isOn = False
                    response.message = "[MatrixModeController]:Resume enable"
            else:
                response.success = True
                response.message = "[MatrixModeController]:Resume disable"
                self._isInit_state = False

        elif request.cmd == "DOCKING":
            if request.arg0 == "ACTIVE":
                if self._isMCUMaster_on_done and self.allowed_master_on_msg.data:
                    self._isDocking_mode = True
                    response.success = True
                    response.message = "[MatrixModeController]:Docking enable"
                    self.get_logger().info("Docking mode trigger")
                    self.current_mode.robot_mode = RobotMode.DOCKING_MODE_ON
                else:
                    self._isDocking_mode = False
                    self._isOn = False
                    response.success = False
                    response.message = "[MatrixModeController]:Docking disable"
                    self._isInit_state = False
                    self.get_logger().error("Press master on done")
            else:
                self._isDocking_mode = False
                self._isOn = False
                response.success = True
                response.message = "[MatrixModeController]:Docking disable"
                self._isInit_state = False

        elif request.cmd == "EMERGENCY_CASE":
            if request.arg0 == "ACTIVE":
                self._isEmergency_case_active = True
                self._isPause = False
                self._isDocking_mode = False
                self._isInit_state = False
                response.success = True
                response.message = "[MatrixModeController]:EMERGENCY_CASE enable"
            else:
                response.success = True
                response.message = "[MatrixModeController]:EMERGENCY_CASE not accept"

        return response

    async def cbDocking(self, request, response):
        srv = ActionController.Request()
        if request.data:
            srv.cmd = "DOCKING"
            srv.arg0 = "ACTIVE"
        else:
            srv.cmd = "NORMAL"
            srv.arg0 = "ACTIVE"

        set_mode_response = await self.cbSetMode(srv, ActionController.Response())
        response.success = set_mode_response.success
        response.message = set_mode_response.message
        return response

    async def cbPause(self, request, response):
        if request.data:
            self._isPause = True
            self._isDocking_mode = False
            self._isEmerCharge_mode = False
            response.success = True
            response.message = "[MatrixModeController]:Pause enable"
        else:
            self._isPause = False
            response.success = True
            response.message = "[MatrixModeController]:Pause disable"
            self._isInit_state = False
        return response

    def clear_state(self):
        self._isPause = False

    def clear_emer_state(self):
        self._isEmergency_case_active = False
        self._isPause = False
        self._isDocking_mode = False

        if self._isMCUMaster_on_done and self._isMasterOnSys:
            req = SetIOs.Request()
            req.cmd = "circuit_reset"
            req.arg0 = 1
            future = self.set_ios_cli.call_async(req)

    def fall(self, current_state, prev_state):
        detect = False
        if current_state != prev_state and not current_state:
            self.get_logger().info("Fall Detect")
            detect = True
        return detect

    def init_state_func(self):
        success = False
        if not self._isMCUMaster_on_done:
            if not self._isMCUMaster_on and self._isSystemReady == Diagnostics.SYSTEM_READY_STATUS_OK:
                self.current_mode.robot_mode = RobotMode.READY_TO_START
                success = True
        else:
            self.current_mode.robot_mode = RobotMode.START_MOTOR
            success = True
        return success

    def check_MasaterOn(self):
        if self.mode_msg.robot_mode == RobotMode.READY_TO_START and self._isMCUMaster_on_done:
            self.current_mode.robot_mode = RobotMode.START_MOTOR

    def state_controller_sequence(self):
        if self.operation_sequence_ == self.operation_sequence.init_io:
            if not hasattr(self, 'smr200_turnon_fan'):
                self.smr200_turnon_fan = False

            self.get_logger().info("[MatrixModeController]:set fan!!!")
            if not self.smr200_turnon_fan and "SMR0200" in self.serial_no:
                self.setIO("cooling_fan", 1)
                self.smr200_turnon_fan = True

            req = SetIOs.Request()
            req.cmd = "use_master_on_system"
            future = self.set_ios_cli.call_async(req)
            future.add_done_callback(self.use_master_on_system_callback)

            self.operation_sequence_ = self.operation_sequence.check_sysready

        elif self.operation_sequence_ == self.operation_sequence.check_sysready:
            self.check_error_device()
            self.operation_sequence_ = self.operation_sequence.check_emer_charge

        elif self.operation_sequence_ == self.operation_sequence.check_emer_charge:
            self.check_EmerCharge()
            if not self._isMasterOnSys and self.fall(self._isEmergency_state, self._isEmergency_state_prev) and self._isMasterOnSys:
                req = SetIOs.Request()
                req.cmd = "circuit_reset"
                req.arg0 = 0
                self.set_ios_cli.call_async(req)

            self._isEmergency_state_prev = self._isEmergency_state
            self.operation_sequence_ = self.operation_sequence.check_button_state

        elif self.operation_sequence_ == self.operation_sequence.check_button_state:
            if self._isSystemReady_comeup:
                self.check_ButtonState()
            self.operation_sequence_ = self.operation_sequence.pub_botton_state

        elif self.operation_sequence_ == self.operation_sequence.pub_botton_state:
            self.operation_sequence_ = self.operation_sequence.control_state

        elif self.operation_sequence_ == self.operation_sequence.control_state:
            self.mode_manager3()
            self.operation_sequence_ = self.operation_sequence.check_sysready

    def use_master_on_system_callback(self, future):
        try:
            response = future.result()
            self._isMasterOnSys = response.text != "0"
        except Exception as e:
            self.get_logger().error("Error call cmd: use_master_on_system")
            self._isMasterOnSys = True

    def check_error_device(self):
        return self._isErrorDevice

    def check_EmerCharge(self):
        if self._isMCUEmergency_charge or self._isSOFTWAREEmergency_charge:
            self._isEmerCharge_state = True
            if self._isMCUMaster_on_done and self._isMasterOnSys:
                req = SetIOs.Request()
                req.cmd = "circuit_reset"
                req.arg0 = 1
                self.set_ios_cli.call_async(req)
        else:
            if self._isEmerCharge_state and self._isMasterOnSys:
                self.get_logger().info("off circuit reset")
                req = SetIOs.Request()
                req.cmd = "circuit_reset"
                req.arg0 = 0
                self.set_ios_cli.call_async(req)
            self._isEmerCharge_state = False

    def check_anotherAutocharge(self):
        self._isAnotherAutocharge = False
        if self._isCharging and not (self._isMCUEmergency_charge or self._isSOFTWAREEmergency_charge) and not self._isDocking_mode:
            self._isAnotherAutocharge = True

    def check_ButtonState(self):
        if self._isMCUEmergency or self._isMCUEmergencyPlugin:
            self._isInit_state = False
            self._isEmergency_case_active = False
            self._isOperating = False
            self._isEmergency_state = True
            self._isOn = False
            self.current_mode.robot_mode = RobotMode.EMERGENCY
        else:
            if not self._isMasterOnSys:
                self._isEmergency_state = False

            if not self._isMCUMaster_on_done:
                self._isEmergency_state = False
                if self._isSystemReady == Diagnostics.SYSTEM_READY_STATUS_OK and self._isMasterOnSys:
                    self.current_mode.robot_mode = RobotMode.READY_TO_START
                    self._isOn = False
            else:
                if not self._isOn:
                    self.current_mode.robot_mode = RobotMode.START_MOTOR
                    self._isOn = True
                    self._isInit_state = True

    def mode_manager3(self):
        self.allowed_master_on_msg.data = False
        self.before_robot_mode = self.mode_msg.robot_mode

        if self._isSystemReady == Diagnostics.SYSTEM_READY_STATUS_OK:
            if not self._isEmerCharge_state:
                if not self._isAnotherAutocharge:
                    if not self._isEmergency_state:
                        if not self._isEmergency_case_active:
                            if not self._isPause:
                                self.mode_msg.robot_mode = self.current_mode.robot_mode
                                self.allowed_master_on_msg.data = True
                            else:
                                self.mode_msg.robot_mode = RobotMode.PAUSE
                                self._isInit_state = False
                        else:
                            self.mode_msg.robot_mode = RobotMode.EMERGENCY_CASE_ACTIVE
                            self._isInit_state = False
                    else:
                        self.clear_emer_state()
                        self.mode_msg.robot_mode = RobotMode.EMERGENCY
                        self._isInit_state = False
                else:
                    self.clear_emer_state()
                    self.mode_msg.robot_mode = RobotMode.CHARGER_ON
                    self._isInit_state = False
            else:
                self.clear_emer_state()
                self.mode_msg.robot_mode = RobotMode.EMERGENCY_CHARGE
                self._isInit_state = False
        elif self._isSystemReady == Diagnostics.SYSTEM_READY_WAIT_COMEUP:
            pass
        else:
            self.clear_emer_state()
            self.mode_msg.robot_mode = RobotMode.ERROR_DEVICE
            self._isInit_state = False

        if self.before_robot_mode != self.mode_msg.robot_mode:
            self.prev_robot_mode = self.before_robot_mode

        if self.mode_msg.robot_mode == RobotMode.READY_TO_START:
            if abs(self.twist.linear.x) <= 0.1 and abs(self.twist.angular.z) <= 0.1:
                self.allowed_master_on_msg.data = True
        elif self.mode_msg.robot_mode == RobotMode.START_MOTOR and self.prev_robot_mode == RobotMode.PAUSE:
            if not self._isInit_motor and abs(self.twist.linear.x) <= 0.1 and abs(self.twist.angular.z) <= 0.1:
                self.allowed_master_on_msg.data = True
                self._isInit_motor = True

        self.pub_mode.publish(self.mode_msg)
        self.pub_allowed_master_on.publish(self.allowed_master_on_msg)
        self.get_logger().info(f"[matrix_mode_contrller]: current robot mode {self.mode_msg.robot_mode}")

    def setIO(self, cmd, value):
        req = SetIOs.Request()
        req.cmd = cmd
        req.arg0 = value
        future = self.set_ios_cli.call_async(req)
        return future

def main(args=None):
    rclpy.init(args=args)

    try:
        controller = MatrixModeController()

        # Use MultiThreadedExecutor to handle callbacks in parallel
        executor = MultiThreadedExecutor()
        executor.add_node(controller)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            controller.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
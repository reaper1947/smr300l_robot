#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from next2_msgs.msg import RobotMode, Diagnostics, Devices, DiagnosticsModern, SeerRobotIOStatus
from next2_msgs.srv import ActionController, SetIOs
from diagnostic_msgs.msg import DiagnosticArray
from enum import IntEnum
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
from sensor_msgs.msg import BatteryState, Temperature
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from rcl_interfaces.msg import SetParametersResult

class OperationSequence(IntEnum):
    INIT_IO                = 0
    CHECK_SYSREADY         = 1
    CHECK_BUTTON_STATE     = 2
    CHECK_EMER_CHARGE      = 3
    PUB_BUTTON_STATE       = 4
    CONTROL_STATE          = 5

class ModePublisher(Node):
    def __init__(self):
        super().__init__('mode_publisher')
        #pub
        self.mode_pub = self.create_publisher(RobotMode, '/robot_mode', 10)
        self.allowed_master_on_pub = self.create_publisher(Bool, '/allowed_master_on', 10)
        #sub
        self.diag_system_ready_sub = self.create_subscription(Diagnostics, '/diagnostics_system_ready', self.system_ready, 10)
        self.emmer_state_sub = self.create_subscription(Bool, '/emergency_io', self.emergency_callback, 10)
        self.battery_state_sub = self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(SeerRobotIOStatus, '/status_io', self.io_status_cb, 10)
        # srv
        self.set_mode_service = self.create_service(ActionController, 'set_mode', self.set_mode)
        self.action_controller_srv = self.create_client(ActionController, 'action_controller')

        self.pause_service = self.create_service(SetBool, 'pause_mode', self.pause_callback)
        self.set_bool_srv = self.create_client(SetBool, 'cancel_action')

        self.docking_service = self.create_service(SetBool, 'docking_mode', self.docking)

        self.set_ios = self.create_client(SetIOs, 'io_service')
        #init
        self.sequence               = OperationSequence.INIT_IO
        self.allowed_master_on_msg  = Bool()
        self.current_mode           = RobotMode()
        self.mode_msg               = RobotMode()
        self.twist                  = Twist()
        self.current_mode.robot_mode = 1
        #bool
        self.device_status                  = False
        self.emer_status                    = True
        self.isInit_state                   = False
        self.isInit_motor                   = False
        self.isSystemReady_comeup           = False
        self.isErrorDevice                  = False
        self.isEmergency_state              = False
        self.isEmergency_state_prev         = False
        self.isEmerCharge_state             = False
        self.isEmergency_case_active        = False
        self.nable_emergency_charge_state   = False
        self.isDocking_mode                 = False
        self.isOperating                    = False
        self.isShutdown                     = False
        self.isCharging                     = False
        self.isPause                        = False
        self.isOn                           = False
        self.isMasterOnSys                  = False
        self.isBumperPush                   = False
        self.isAnotherAutocharge            = False
        self.permissive_on_when_bumper_detect = True
        self.isMCUMaster_on                 = False
        self.isMCUMaster_on_done            = False
        self.isMCUEmergency                 = False
        self.isMCUEmergencyPlugin           = False
        self.isMCUEmergency_charge          = False # manual charge
        self.isMCUComputer_ready            = False
        self.isMCUShutdown_robot            = False
        self.isMCUEnable_charger            = False
        self.isMCUBumper                    = False
        self.isSOFTWAREEmergency_charge     = False
        self.isEmerCharge_mode              = False
        # int
        self.charge_state                   = 0
        self.prev_robot_mode                = 0
        self.before_robot_mode              = 0
        self.isSystemReady                  = 0
        self.isEmerState_mode               = 0
        # param
        self.declare_parameter('robot_mode', 0)
        self.declare_parameter('enable_emergency_charge_state', True)
        self.declare_parameter('emergency_charge_active_value', 0)
        self.declare_parameter('emergency_plugin_active_value', 0)
        self.declare_parameter('emergency_base_active_value', 0)
        self.declare_parameter('stop_movement', 0)
        self.declare_parameter('stop_all', 0)

        self.mode = self.get_parameter('robot_mode').value
        self.enable_emergency_charge_state = self.get_parameter('enable_emergency_charge_state').value
        self.emergency_charge_active_value = self.get_parameter('emergency_charge_active_value').value
        self.emergency_plugin_active_value = self.get_parameter('emergency_plugin_active_value').value
        self.emergency_base_active_value = self.get_parameter('emergency_base_active_value').value
        self.stop_movement = self.get_parameter('stop_movement').value
        self.stop_all = self.get_parameter('stop_all').value

        self.timer = self.create_timer(0.1, self.run_state_machine)

    def odom_callback(self, msg):
        self.twist = msg.twist.twist

    def io_status_cb(self, msg: SeerRobotIOStatus):
        input_io = [io.status for io in msg.io_inputs]
        output_io = [io.status for io in msg.io_outputs]
        # input
        master_on_push = bool(input_io[3] if input_io else 1)
        self.isMCUEmergency_charge = master_on_push
        self.isMCUEnable_charger = master_on_push
        self.isMCUMaster_on = master_on_push
        self.isMasterOnSys = master_on_push 
        # self.isMCUEmergency = self.invertMCUState(input_io[1], self.emergency_base_active_value)
        # self.isMCUMaster_on = self.invertMCUState(input_io[2])
        # self.isMCUEmergency_charge = self.invertMCUState(input_io[4], self.emergency_charge_active_value) if self.enable_emergency_charge_state else False
        # self.isMCUEmergencyPlugin = self.invertMCUState(input_io[4], self.emergency_plugin_active_value)

        # output
        self.relay_on = bool(output_io[0] == 1 and output_io[1] == 1)
        self.isMCUMaster_on_done = self.relay_on

        # self.isMCUComputer_ready = self.invertMCUState(output_io[0], 1)
        # self.isMCUShutdown_robot = self.invertMCUState(output_io[1], 1)
        # self.isMCUEnable_charger = self.invertMCUState(output_io[3], 1)

    def system_ready(self, msg): # check diagnostic readyyy!!!!
        self.isSystemReady = msg.system_ready
        self.isSystemReady_comeup = True
        if self.isSystemReady == 0:
            self.isErrorDevice = False
        elif self.isSystemReady == 2:
            self.isErrorDevice = True

    def run_state_machine(self):
        match self.sequence:
            case OperationSequence.INIT_IO:
                # self.get_logger().info("INIT_IO")
                # try:
                #     if not self.set_ios.wait_for_service(timeout_sec=1.0):
                #         self.get_logger().warn("Service not available")
                #         return
                    
                #     srv = SetIOs.Request()
                #     srv.cmd = "use_master_on_system"
                #     future = self.set_ios.call_async(srv)
                #     rclpy.spin_until_future_complete(self, future)

                #     if future.done():
                #         response = future.result()
                #         if response.text == "0":
                #             self.isMasterOnSys = False
                #             print("isMasterOnSys", self.isMasterOnSys)
                #         else:
                #             self.isMasterOnSys = True
                #             print("isMasterOnSys", self.isMasterOnSys)
                # except Exception as e:
                #     self.get_logger().error(f"Failed to process msg: {e}")
                # self.isMasterOnSys = True
                self.sequence = OperationSequence.CHECK_SYSREADY

            case OperationSequence.CHECK_SYSREADY: # check diagnostic
                # self.get_logger().info("CHECK_SYSREADY")
                if self.isErrorDevice:
                    # self.current_mode.robot_mode = RobotMode.ERROR_DEVICE
                    self.sequence = OperationSequence.CHECK_EMER_CHARGE
                else:
                    # self.isMasterOnSys = False
                    self.sequence = OperationSequence.CHECK_EMER_CHARGE

            case OperationSequence.CHECK_EMER_CHARGE: # check emer charge
                # self.get_logger().info("CHECK_EMER_CHARGE")
                self.check_emer_charge()
                if not self.isMasterOnSys:
                    if (self.fall(self.isEmergency_state, self.isEmergency_state_prev)) and self.isMasterOnSys:
                        srv = SetIOs.Request()
                        srv.cmd = "circuit_reset"
                        srv.arg0 = 0
                        result = self.set_ios.call_async(srv)
                        rclpy.spin_until_future_complete(self, result)
                    self.isEmergency_state_prev = self.isEmergency_state
                self.sequence = OperationSequence.CHECK_BUTTON_STATE

            case OperationSequence.CHECK_BUTTON_STATE: # check emergency
                # self.get_logger().info("CHECK_BUTTON_STATE")
                if self.isSystemReady_comeup:
                    # print('isSystemReady_comeup',self.isSystemReady_comeup)
                    self.check_button()
                self.sequence = OperationSequence.PUB_BUTTON_STATE

            case OperationSequence.PUB_BUTTON_STATE:
                # self.get_logger().info("PUB_BUTTON_STATE")
                self.sequence = OperationSequence.CONTROL_STATE

            case OperationSequence.CONTROL_STATE:
                # self.get_logger().info("CONTROL_STATE")
                self.mode_manager()
                self.sequence = OperationSequence.CHECK_SYSREADY

            # case _:
            #     self.get_logger().warn("Unknown state. Resetting...")
            #     self.sequence = OperationSequence.INIT_IO
          
    def battery_callback(self, msg):
        self.isEmerState_mode = msg.power_supply_status
        if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            self.isCharging = True
        else:
            self.isCharging = False

    def check_emer_charge(self):
        if self.isMCUEmergency_charge or self.isSOFTWAREEmergency_charge:
            self.isEmerCharge_state = True
            if self.isMCUMaster_on_done and self.isMasterOnSys:
                srv = SetIOs.Request()
                srv.cmd = "circuit_reset"
                srv.arg0 = 1
                result = self.set_ios.call_async(srv)
                rclpy.spin_until_future_complete(self, result)
        else:
            if self.isEmerCharge_state and self.isMasterOnSys:
                srv = SetIOs.Request()
                srv.cmd = "circuit_reset"
                srv.arg0 = 0
                result = self.set_ios.call_async(srv)
                rclpy.spin_until_future_complete(self, result)
            self.isEmerCharge_state = False

    def fall(self, current_state, prev_state):
        detect = False
        if current_state != prev_state:
            if not current_state:
                self.get_logger().info(f"Fall Detect")
                detect = True
        return detect
    
    def emergency_callback(self, msg):
        emer_data = msg.data
        self.isMCUEmergency = not emer_data
        # print(self.isMCUEmergency)

    def check_button(self):
        if self.isMCUEmergency or self.isMCUEmergencyPlugin:
            self.isInit_state = False
            self.isEmergency_case_active = False
            self.isOperating = False
            self.isEmergency_state = True
            self.isOn = False
            self.current_mode.robot_mode = RobotMode.EMERGENCY
        else:
            self.current_mode.robot_mode = RobotMode.IDLE
            if not self.isMasterOnSys:
                self.isEmergency_state = False
            if not self.isMCUMaster_on_done:
                self.isEmergency_state = False
                if self.isSystemReady == 0:
                    if self.isMasterOnSys:
                        self.current_mode.robot_mode = RobotMode.READY_TO_START
                    else:
                        self.isOn = False
            else:
                # if self.isMCUMaster_on_done and not self.isOn:
                if self.isMCUMaster_on_done:
                    self.current_mode.robot_mode = RobotMode.START_MOTOR
                    self.isOn = True
                    self.isInit_state = True
        
        # print('isMasterOnSys', self.isMasterOnSys)
        # print('isMCUMaster_on_done', self.isMCUMaster_on_done)
    
    def clear_emer_state(self):
        self.isEmergency_case_active = False
        self.isPause = False
        self.isDocking_mode = False

        if (self.isMCUMaster_on_done and self.isMasterOnSys):
            srv = SetIOs.Request()
            srv.cmd = "circuit_reset"
            srv.arg0 = 1
            future = self.set_ios.call_async(srv)
            rclpy.spin_until_future_complete(self, future)

    def mode_manager(self):
        self.allowed_master_on_msg.data = False
        self.before_robot_mode = self.mode_msg.robot_mode
        # print('before_robotmode', self.before_robot_mode)
        # print('isSystemReady', self.isSystemReady)
        # print('current_mode', self.current_mode.robot_mode)
        if self.isSystemReady == 0:
            # print('isSystemReady', self.isSystemReady)
            if not self.isEmerCharge_state:
                # print('isEmerCharge_state', self.isEmerCharge_state)
                if not self.isAnotherAutocharge:
                    # print('isAnotherAutocharge', self.isAnotherAutocharge)
                    if not self.isEmergency_state:
                        # print('isEmergency_state', self.isAnotherAutocharge)
                        self.mode_msg.robot_mode = RobotMode.IDLE
                        if not self.isEmergency_case_active:
                            # print('isEmergency_case_active', self.isEmergency_case_active)
                            if not self.isPause:
                                # print('isPause', self.isPause)
                                self.mode_msg.robot_mode = self.current_mode.robot_mode
                                self.allowed_master_on_msg.data = True
                            else:
                                self.mode_msg.robot_mode = RobotMode.PAUSE
                                self.isInit_state = False
                        else:
                            self.mode_msg.robot_mode = RobotMode.EMERGENCY_CASE_ACTIVE
                            self.isInit_state = False
                    else:
                        self.clear_emer_state()
                        self.mode_msg.robot_mode = RobotMode.EMERGENCY
                        self.isInit_state = False
                else:
                    self.clear_emer_state()
                    self.mode_msg.robot_mode = RobotMode.CHARGER_ON
                    self.isInit_state = False
            else:
                self.clear_emer_state()
                self.mode_msg.robot_mode = RobotMode.EMERGENCY_CHARGE
                self.isInit_state = False
        elif self.isSystemReady == 3:
            self.get_logger().info("SYSTEM_READY_WAIT_COMEUP")
        else:
            # print('SYSTEM_READY_ERROR')
            self.clear_emer_state()
            self.mode_msg.robot_mode = RobotMode.ERROR_DEVICE
            self.isInit_state = False
        # print('mode_msg', self.mode_msg.robot_mode)

        if self.before_robot_mode != self.mode_msg.robot_mode:
            # self.prev_robot_mode = self.before_robot_mode
            self.prev_robot_mode = 1

        if self.mode_msg.robot_mode == RobotMode.READY_TO_START:
            if abs(self.twist.linear.x) <= 0.1 and abs(self.twist.angular.z) <= 0.1:
                self.allowed_master_on_msg.data = True
            else:
                self.allowed_master_on_msg = False
        elif self.mode_msg.robot_mode == RobotMode.START_MOTOR and self.prev_robot_mode == RobotMode.PAUSE:
            if not self.isInit_motor:
                if abs(self.twist.linear.x) <= 0.1 and abs(self.twist.angular.z) <= 0.1:
                    self.allowed_master_on_msg.data = True
                    self.isInit_motor = True
                else:
                    self.allowed_master_on_msg.data = False
            # else:
            #     self.get_logger().info("isInit_motor")
        else:
            self.isInit_motor = False

        # pub
        self.mode_pub.publish(self.mode_msg)
        self.allowed_master_on_pub.publish(self.allowed_master_on_msg)
        self.get_logger().info(f"Published mode: {self.mode_msg.robot_mode}")
        self.set_parameters([Parameter('robot_mode', Parameter.Type.INTEGER, int(self.mode_msg.robot_mode))])

    def invertMCUState(self, signal, active_signal):
        if active_signal == -1:
            return False
        else:
            if signal == active_signal:
                return True
            else:
                return False
            
    # service
    def callback(self, params):
        srv = ActionController.Request()
        for param in params:
            if param.name == 'stop_movement':
                self.stop_movement = param.value
            elif param.name == 'stop_all':
                self.stop_all = param.value
            elif param.name == 'robot_mode':
                self.mode = param.value

        if self.stop_movement:
            srv.cmd = "STOP_MOVEMENT"
            self.executor.create_task(self.set_mode(srv))
            self.set_parameters([Parameter('stop_movement', Parameter.Type.BOOL, False)])
        
        elif self.stop_all:
            srv.cmd = "STOP_ALL"
            self.executor.create_task(self.set_mode(srv))
            self.set_parameters([Parameter('stop_all', Parameter.Type.BOOL, False)])

        elif self.mode == 0:
            srv.cmd = "NORMAL"
            srv.arg0 = "ACTIVE"
            # self.executor.create_task(self.set_mode(srv))

        elif self.mode == RobotMode.DOCKING_MODE_ON:
            srv.cmd = "DOCKING"
            srv.arg0 = "ACTIVE"
            # self.executor.create_task(self.set_mode(srv))

        elif self.mode == RobotMode.PAUSE:
            srv.cmd = "PAUSE"
            srv.arg0 = "ACTIVE"
            # self.executor.create_task(self.set_mode(srv))
        
        elif self.mode == RobotMode.RESUME:
            srv.cmd = "PAUSE"
            srv.arg0 = "INACTIVE"
            # self.executor.create_task(self.set_mode(srv))

        elif self.mode == RobotMode.EMERGENCY_CASE_ACTIVE:
            srv.cmd = "EMERGENCY_CASE"
            srv.arg0 = "ACTIVE"
            # self.executor.create_task(self.set_mode(srv))
            
        self.executor.create_task(self.set_mode(srv))
        return SetParametersResult(successful=True)

    def pause_callback(self, request, response): # pause_service
        if request.data:
            self.isPause = True
            self.isDocking_mode = False
            self.isEmerCharge_mode = False
            response.success = True
            response.message = "Pause enable"
        else:
            self.isPause = False
            self.isInit_state = False
            response.success = False
            response.message = "Pause disable"
        return response

    def set_mode(self, request, response): # set_mode_service
        if request.cmd == 'NORMAL':
            if request.arg0 == "ACTIVE":
                self.isPause = False
                self.isDocking_mode = False
                self.isEmergency_case_active = False
                self.isOn = False
                self.isInit_state = False
                response.success = True
                response.message = "Normal mode"
            else:
                response.success = True
                response.message = "Normal mode disable"

        elif request.cmd == 'STOP_MOVEMENT':
            srv = ActionController.Request()
            srv.cmd = "action_cancel"
            srv.action_name = "movement"
            srv.arg0 = ""
            srv.arg1 = ""

            future = self.action_controller_srv.call_async(srv)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result:
                response.success = result.success
                response.message = result.message
            else:
                response.success = False
                response.message = "Service call failed or timed out."

        elif request.cmd == 'STOP_ALL':
            cancel_req = ActionController.Request()
            cancel_req.cmd = "action_cancel"
            cancel_req.action_name = "STOP_ALL"
            cancel_req.arg0 = ""
            cancel_req.arg1 = ""

            future = self.action_controller_srv.call_async(cancel_req)
            rclpy.spin_until_future_complete(self, future)
            cancel_result = future.result()

            if cancel_result:
                response.success = cancel_result.success
                response.message = cancel_result.message
            else:
                response.success = False
                response.message = "STOP_ALL service call failed"

            bool_req = SetBool.Request()
            bool_req.data = True
            bool_future = self.set_bool_srv.call_async(bool_req)
            rclpy.spin_until_future_complete(self, bool_future)
            bool_result = bool_future.result()

            if bool_result:
                if not bool_result.success:
                    response.success = False
                response.message = bool_result.message
            else:
                response.success = False
                response.message = "SetBool service call failed"

            self.get_logger().warn("STOP_ALL issued")

        elif request.cmd == 'PAUSE':
            if request.arg0 == "ACTIVE":
                self.isPause = True
                response.success = True
                response.message = "Pause enabled"
            else:
                self.isPause = False
                response.success = True
                response.message = "Pause disabled"

        elif request.cmd == 'RESUME':
            if request.arg0 == "ACTIVE":
                if self.permissive_on_when_bumper_detect:
                    self.isPause = True
                    self.isInit_state = False
                    self.isOn = True
                    response.success = True
                    response.message = "Resume enable (with permissive bumper)"
                else:
                    if self.isBumperPush:
                        self.isPause = False
                        self.isInit_state = False
                        self.isOn = False
                        response.success = True
                        response.message = "Resume enable"
                    else:
                        response.success = False
                        response.message = "Resume denied: bumper not pushed"
            else:
                self.isInit_state = False
                response.success = True
                response.message = "Resume disable"

        elif request.cmd == 'DOCKING':
            if request.arg0 == "ACTIVE":
                if self.isMCUMaster_on_done and self.allowed_master_on_msg.data:
                    self.isDocking_mode = True
                    self.current_mode.robot_mode = RobotMode.DOCKING_MODE_ON
                    self.isOn = False
                    response.success = True
                    response.message = "Docking enabled"
                    self.get_logger().info("Docking mode triggered")
                else:
                    self.isDocking_mode = False
                    self.isOn = False
                    response.success = False
                    response.message = "Docking denied: press master on done"
                    self.get_logger().error(response.message)
            else:
                self.isDocking_mode = False
                self.isOn = False
                self.isInit_state = False
                response.success = True
                response.message = "Docking disabled"

        elif request.cmd == 'EMERGENCY_CASE':
            if request.arg0 == "ACTIVE":
                self.isEmergency_case_active = True
                self.isPause = False
                self.isDocking_mode = False
                self.isInit_state = False
                response.success = True
                response.message = "EMERGENCY_CASE enabled"
            else:
                response.success = True
                response.message = "EMERGENCY_CASE not accepted"

        else:
            response.success = False
            response.message = f"Unknown command: {request.cmd}"

        return response

    def docking(self, request, response): #docking service
        srv = ActionController.Request()
        if request.data:
            srv.cmd = "DOCKING"
            srv.arg0 = "ACTIVE"
        else:
            srv.cmd = "NORMAL"
            srv.arg0 = "ACTIVE"
        self.set_mode(srv, response)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ModePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

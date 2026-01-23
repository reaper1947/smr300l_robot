#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from enum import IntEnum
from next2_msgs.msg import RobotMode, Diagnostics, SeerRobotIOStatus
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

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

        # Publishers
        self.mode_pub = self.create_publisher(RobotMode, 'robot_mode', 10)
        self.allowed_master_on_pub = self.create_publisher(Bool, 'allowed_master_on', 10)

        # Subscribers
        self.create_subscription(Diagnostics, '/diagnostics_system_ready', self.system_ready_cb, 10)
        self.create_subscription(SeerRobotIOStatus, '/status_io', self.io_status_cb, 10)
        self.create_subscription(Bool, 'emergency_io', self.emergency_cb, 10)
        self.create_subscription(BatteryState, 'battery_state', self.battery_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Parameters
        self.declare_parameter('robot_mode', 0)
        self.declare_parameter('enable_emergency_charge_state', True)
        self.declare_parameter('emergency_charge_active_value', 0)
        self.declare_parameter('emergency_plugin_active_value', 0)
        self.declare_parameter('emergency_base_active_value', 0)
        self.declare_parameter('stop_movement', 0)
        self.declare_parameter('stop_all', 0)

        # States and flags
        self.sequence = OperationSequence.INIT_IO
        self.robot_mode = RobotMode()
        self.robot_mode.robot_mode = RobotMode.IDLE
        self.allowed_master_on_msg = Bool()
        self.twist = Twist()

        # Status flags
        self.isSystemReady      = 1
        self.charge_manual      = 0
        self.isEmerCharge_state = 0

        self.isMasterOnSys              = False
        self.isMCUMaster_on_done        = False
        self.emer_status                = True
        self.isCharging                 = False
        self.relay_on                   = False
        self.isDocking_mode             = False
        self.allowed_master_on_msg.data = False

        self.input_io = []
        self.output_io = []

        # Timer
        self.timer = self.create_timer(0.5, self.run_state_machine)

    def run_state_machine(self):
        match self.sequence:
            case OperationSequence.INIT_IO:
                # self.get_logger().info("INIT_IO")
                self.sequence = OperationSequence.CHECK_SYSREADY

            case OperationSequence.CHECK_SYSREADY: # check diagnostic
                # self.get_logger().info("CHECK_SYSREADY")
                if self.isSystemReady == 2:
                    self.robot_mode.robot_mode = RobotMode.ERROR_DEVICE
                    self.isMasterOnSys = False
                    self.allowed_master_on_msg.data = False
                elif self.isSystemReady == 0:
                    self.isMasterOnSys = False
                    self.sequence = OperationSequence.CHECK_EMER_CHARGE

            case OperationSequence.CHECK_EMER_CHARGE:
                # self.get_logger().info("CHECK_EMER_CHARGE")
                if self.charge_manual == 1 or self.isDocking_mode == True:
                    self.isMasterOnSys = False
                    self.allowed_master_on_msg.data = False
                    if self.isCharging:
                        self.robot_mode.robot_mode = RobotMode.CHARGER_ON
                    else:
                        self.robot_mode.robot_mode = RobotMode.CHARGER_OFF
                else:
                    self.sequence = OperationSequence.CHECK_BUTTON_STATE

            case OperationSequence.CHECK_BUTTON_STATE:
                # self.get_logger().info("CHECK_BUTTON_STATE")
                if self.emer_status == False:
                    self.robot_mode.robot_mode = RobotMode.EMERGENCY
                    self.isMasterOnSys = False
                    self.allowed_master_on_msg.data = False
                else:
                    self.sequence = OperationSequence.PUB_BUTTON_STATE

            case OperationSequence.PUB_BUTTON_STATE:
                # self.get_logger().info("PUB_BUTTON_STATE")
                if self.relay_on:
                    self.isMasterOnSys = True
                    self.sequence = OperationSequence.CONTROL_STATE
                else:
                    self.isMasterOnSys = False
                    self.sequence = OperationSequence.CONTROL_STATE

            case OperationSequence.CONTROL_STATE:
                # self.get_logger().info("CONTROL_STATE")
                if self.isMasterOnSys:
                    self.robot_mode.robot_mode = RobotMode.READY_TO_START
                    self.allowed_master_on_msg.data = True
                    self.isMCUMaster_on_done = True
                    self.check_master_on()
                else:
                    self.robot_mode.robot_mode = RobotMode.IDLE
                    self.isMCUMaster_on_done = False
                    self.allowed_master_on_msg.data = False
                self.sequence = OperationSequence.CHECK_SYSREADY

            # case _:
            #     self.get_logger().warn("Unknown state. Resetting...")
            #     self.sequence = OperationSequence.INIT_IO
            
        # Publish robot mode and allowed master state
        self.mode_pub.publish(self.robot_mode)
        self.allowed_master_on_pub.publish(self.allowed_master_on_msg)
        self.set_parameters([Parameter('robot_mode', Parameter.Type.INTEGER, int(self.robot_mode.robot_mode))])
        self.get_logger().info(f"Published mode: {self.robot_mode.robot_mode}")

    def system_ready_cb(self, msg: Diagnostics):
        self.isSystemReady = msg.system_ready

    def io_status_cb(self, msg: SeerRobotIOStatus):
        self.input_io = [io.status for io in msg.io_inputs]
        self.output_io = [io.status for io in msg.io_outputs]

        self.charge_manual = self.input_io[3] if self.input_io else 1
        if self.charge_manual == 1:
            self.robot_mode.robot_mode = RobotMode.EMERGENCY_CHARGE

        self.relay_on = bool(self.output_io[0] == 1 and self.output_io[1] == 1)

    def emergency_cb(self, msg: Bool):
        self.emer_status = msg.data
        
    def battery_cb(self, msg: BatteryState):
        self.isEmerCharge_state = msg.power_supply_status
        if self.isEmerCharge_state == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            self.isCharging = True
        else:
            self.isCharging = False
        # self.isCharging = msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING

    def odom_cb(self, msg: Odometry):
        self.twist = msg.twist.twist

    def check_master_on(self):
        if self.robot_mode.robot_mode == RobotMode.READY_TO_START and self.isMCUMaster_on_done:
            self.robot_mode.robot_mode = RobotMode.START_MOTOR
        else:
            self.sequence = OperationSequence.CHECK_SYSREADY

def main(args=None):
    rclpy.init(args=args)
    node = ModePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

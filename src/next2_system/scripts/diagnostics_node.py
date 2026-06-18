#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, LaserScan, PointCloud, Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from next2_msgs.msg import Diagnostics, Devices, DiagnosticsModern
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class DiagnosticsCheck(Node):
    def __init__(self):
        super().__init__('diagnostics_node')

        # Declare and read parameters
        self.use_diagnostics = self.declare_parameter('use_diagnostics', True).value

        self.check_bms = self.declare_parameter('check_bms', False).value
        bms_topic = self.declare_parameter('bms_topic', 'battery_state').value
        self.check_io = self.declare_parameter('check_io', True).value
        io_topic = self.declare_parameter('io_topic', 'io_heartbeat').value
        self.check_lidar = self.declare_parameter('check_lidar', False).value
        lidar_topic = self.declare_parameter('lidar_topic', 'scan').value           # lidar_front_slamkit
        self.check_motor = self.declare_parameter('check_motor', False).value
        motor_topic = self.declare_parameter('motor_topic', 'odom').value
        self.check_cam = self.declare_parameter('check_cam', False).value
        cam_topic = self.declare_parameter('cam_topic', 'point_cloud').value
        self.check_xiao = self.declare_parameter('check_xiao', False).value
        xiao_topic = self.declare_parameter('xiao_topic', 'xiao_heartbeat').value
        self.check_imu = self.declare_parameter('check_imu', False).value
        imu_topic = self.declare_parameter('imu_topic', '/imu/data_raw').value
        self.check_dmx = self.declare_parameter('check_dmx', False).value
        dmx_topic = self.declare_parameter('dmx_topic', '/dmx_heartbeat').value     # led

        # Publisher
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.diag_system_ready = self.create_publisher(Diagnostics, '/diagnostics_system_ready', 10)
        self.diag_modern = self.create_publisher(DiagnosticsModern, '/diagnostics_modern', 10)

        # subscriptions
        self.create_subscription(BatteryState, bms_topic, self.bms_callback, 5)
        self.create_subscription(Bool, io_topic, self.io_callback, 10)
        self.create_subscription(LaserScan, lidar_topic, self.lidar_callback, 10)
        self.create_subscription(Odometry, motor_topic, self.motor_callback, 10)
        self.create_subscription(PointCloud, cam_topic, self.cam_callback, 10)
        self.create_subscription(Bool, xiao_topic, self.xiao_callback, 10)
        self.create_subscription(Bool, imu_topic, self.imu_callback, 10)
        self.create_subscription(Bool, dmx_topic, self.dmx_callback, 10)

        # Last message times
        self.last_times = {
            "BMS": None,
            "IO": None,
            "LIDAR_front": None,
            "MOTOR": None,
            "CAM": None,
            "XIAO": None,
            "IMU": None,
            "DMX": None
        }

        self.timer = self.create_timer(0.1, self.pub_diagnostic)

        # init
        self.msg = DiagnosticArray()
        self.msg.header.stamp = self.get_clock().now().to_msg()

        self.msg_device = Devices()
        self.msg_device.lidar_front = 1
        self.msg_device.lidar_rear  = 1
        self.msg_device.imu         = 1
        self.msg_device.camera_1    = 1
        self.msg_device.camera_2    = 1
        self.msg_device.camera_3    = 1
        self.msg_device.camera_rear = 1
        self.msg_device.bms         = 1
        self.msg_device.bms_base    = 1
        self.msg_device.bms_plugin  = 1
        self.msg_device.mcu_1       = 1 # xiao
        self.msg_device.mcu_2       = 1 # io
        self.msg_device.motor       = 1
        self.msg_device.zigbee      = 1
        self.msg_device.dmx         = 1

        self.msg_system_ready = Diagnostics()
        self.msg_system_ready.device_state = self.msg_device

        self.msg_modern = DiagnosticsModern()

    def update_time(self, key):
        self.last_times[key] = self.get_clock().now()

    def bms_callback(self, msg):
        self.update_time("BMS")

    def io_callback(self, msg):
        self.update_time("IO")

    def lidar_callback(self, msg):
        self.update_time("LIDAR_front")

    def motor_callback(self, msg):
        self.update_time("MOTOR")

    def cam_callback(self, msg):
        self.update_time("CAM")

    def xiao_callback(self, msg):
        self.update_time("XIAO")

    def imu_callback(self, msg):
        self.update_time("IMU")

    def dmx_callback(self, msg):
        self.update_time("DMX")

    def check_timeout(self, name):
        last_time = self.last_times[name]
        if last_time is None:
            return DiagnosticStatus.ERROR, "Never received"
        elapsed = (self.get_clock().now() - last_time).nanoseconds * 1e-9
        if elapsed > 1:
            return DiagnosticStatus.ERROR, f"No message for {int(elapsed)}s"
        else:
            return DiagnosticStatus.OK, "OK"

    def create_status(self, name, hardware_id):
        level, message = self.check_timeout(name)
        status = DiagnosticStatus()
        status.level = level
        status.name = name
        status.hardware_id = hardware_id
        status.message = message
        return status

    def diagnostics_device(self):
        self.msg = DiagnosticArray()
        self.msg.header.stamp = self.get_clock().now().to_msg()

        self.msg_modern.status.clear()
        if self.get_parameter('check_bms').get_parameter_value().bool_value:
            status_bat = self.create_status("BMS", "battery")
            self.msg.status.append(status_bat)
            self.msg_modern.status.append(status_bat)
            self.msg_device.bms = 0 if status_bat.level == DiagnosticStatus.OK else 2
            if status_bat.level != DiagnosticStatus.OK:
                self.msg_system_ready.system_ready = Diagnostics.SYSTEM_READY_STATUS_FAULT
                self.msg_modern.system_ready = DiagnosticsModern.SYSTEM_READY_STATUS_FAULT
        else:
            self.msg_device.bms = 1

        if self.get_parameter('check_io').get_parameter_value().bool_value:
            status_io = self.create_status("IO", "io_device")
            self.msg.status.append(status_io)
            self.msg_modern.status.append(status_io)
            self.msg_device.mcu_2 = 0 if status_io.level == DiagnosticStatus.OK else 2
            if status_io.level != DiagnosticStatus.OK:
                self.msg_system_ready.system_ready = Diagnostics.SYSTEM_READY_STATUS_FAULT
                self.msg_modern.system_ready = DiagnosticsModern.SYSTEM_READY_STATUS_FAULT
        else:
            self.msg_device.mcu_2 = 1

        if self.get_parameter('check_lidar').get_parameter_value().bool_value:
            status_lidar_f = self.create_status("LIDAR_front", "lidar_f")
            self.msg.status.append(status_lidar_f)
            self.msg_modern.status.append(status_lidar_f)
            self.msg_device.lidar_front = 0 if status_lidar_f.level == DiagnosticStatus.OK else 2
            if status_lidar_f.level != DiagnosticStatus.OK:
                self.msg_system_ready.system_ready = Diagnostics.SYSTEM_READY_STATUS_FAULT
                self.msg_modern.system_ready = DiagnosticsModern.SYSTEM_READY_STATUS_FAULT
        else:
            self.msg_device.lidar_front = 1

        if self.get_parameter('check_motor').get_parameter_value().bool_value:
            status_motor = self.create_status("MOTOR", "odom")
            self.msg.status.append(status_motor)
            self.msg_modern.status.append(status_motor)
            self.msg_device.motor = 0 if status_motor.level == DiagnosticStatus.OK else 2
            if status_motor.level != DiagnosticStatus.OK:
                self.msg_system_ready.system_ready = Diagnostics.SYSTEM_READY_STATUS_FAULT
                self.msg_modern.system_ready = DiagnosticsModern.SYSTEM_READY_STATUS_FAULT
        else:
            self.msg_device.motor = 1

        if self.get_parameter('check_cam').get_parameter_value().bool_value:
            status_cam = self.create_status("CAM", "camera1")
            self.msg.status.append(status_cam)
            self.msg_modern.status.append(status_cam)
            self.msg_device.camera_1 = 0 if status_cam.level == DiagnosticStatus.OK else 2
            if status_cam.level != DiagnosticStatus.OK:
                self.msg_system_ready.system_ready = Diagnostics.SYSTEM_READY_STATUS_FAULT
                self.msg_modern.system_ready = DiagnosticsModern.SYSTEM_READY_STATUS_FAULT
        else:
            self.msg_device.camera_1 = 1

        if self.get_parameter('check_xiao').get_parameter_value().bool_value:
            status_xiao = self.create_status("XIAO", "xiao_mcu1")
            self.msg.status.append(status_xiao)
            self.msg_modern.status.append(status_xiao)
            self.msg_device.mcu_1 = 0 if status_xiao.level == DiagnosticStatus.OK else 2
            if status_xiao.level != DiagnosticStatus.OK:
                self.msg_system_ready.system_ready = Diagnostics.SYSTEM_READY_STATUS_FAULT
                self.msg_modern.system_ready = DiagnosticsModern.SYSTEM_READY_STATUS_FAULT
        else:
            self.msg_device.mcu_1 = 1

        if self.get_parameter('check_imu').get_parameter_value().bool_value:
            status_imu = self.create_status("IMU", "imu_slamkit")
            self.msg.status.append(status_imu)
            self.msg_modern.status.append(status_imu)
            self.msg_device.imu = 0 if status_imu.level == DiagnosticStatus.OK else 2
            if status_imu.level != DiagnosticStatus.OK:
                self.msg_system_ready.system_ready = Diagnostics.SYSTEM_READY_STATUS_FAULT
                self.msg_modern.system_ready = DiagnosticsModern.SYSTEM_READY_STATUS_FAULT
        else:
            self.msg_device.imu = 1

        if self.get_parameter('check_dmx').get_parameter_value().bool_value:
            status_dmx = self.create_status("DMX", "dmx_led")
            self.msg.status.append(status_dmx)
            self.msg_modern.status.append(status_dmx)
            self.msg_device.dmx = 0 if status_dmx.level == DiagnosticStatus.OK else 2
            if status_dmx.level != DiagnosticStatus.OK:
                self.msg_system_ready.system_ready = Diagnostics.SYSTEM_READY_STATUS_FAULT
                self.msg_modern.system_ready = DiagnosticsModern.SYSTEM_READY_STATUS_FAULT
        else:
            self.msg_device.dmx = 1

    def pub_diagnostic(self):
        self.diagnostics_device()
        if self.get_parameter('use_diagnostics').get_parameter_value().bool_value:
            for i in self.msg_modern.status:
                if any(ord(status.level) == 2 for status in self.msg_modern.status):
                    self.msg_system_ready.system_ready = Diagnostics.SYSTEM_READY_STATUS_FAULT
                    self.msg_modern.system_ready = DiagnosticsModern.SYSTEM_READY_STATUS_FAULT
                else:
                    self.msg_system_ready.system_ready = Diagnostics.SYSTEM_READY_STATUS_OK
                    self.msg_modern.system_ready = DiagnosticsModern.SYSTEM_READY_STATUS_OK
        else:
            self.msg_system_ready.system_ready = Diagnostics.SYSTEM_READY_NOT_USE
            self.msg_modern.system_ready = DiagnosticsModern.SYSTEM_READY_NOT_USE

        self.msg_system_ready.device_state = self.msg_device
        self.diag_pub.publish(self.msg)
        self.diag_system_ready.publish(self.msg_system_ready)
        self.diag_modern.publish(self.msg_modern)

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsCheck()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

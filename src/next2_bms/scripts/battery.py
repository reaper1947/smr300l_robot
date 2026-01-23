#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
from sensor_msgs.msg import BatteryState, Temperature
from std_msgs.msg import Int16, String
from next2_bms.greenway_bms.greenway_bms import GREENWAY_BMS
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class GreenwayBMSNode(Node):
    def __init__(self):
        super().__init__('greenway_bms_node')
        # Declare and get CAN device parameter
        self.can_device = self.declare_parameter("can_device", "").value
        if not self.can_device:
            self.get_logger().error("Parameter 'can_device' is empty. Node will shutdown.")
            rclpy.shutdown()
            return

        # Initialize BMS interface
        self.bms = GREENWAY_BMS()
        if not self.bms.initBMS(self.can_device, 1.0):
            self.get_logger().error(f"Failed to initialize BMS on CAN device '{self.can_device}'. Node will shutdown.")
            rclpy.shutdown()
            return
        
        # Publishers
        self.pub_battery_state = self.create_publisher(BatteryState, 'battery_state', 10)
        self.pub_temperature = self.create_publisher(Temperature, 'battery_temperature', 10)
        self.pub_battery_alarm = self.create_publisher(String, 'alarm', 10)
        self.pub_battery_cycle = self.create_publisher(Int16, 'battery_charge_cycles', 10)

        # Initialize message instances
        self.battery_msg = BatteryState()
        self.battery_temp_msg = Temperature()
        self.discharge_cycles = Int16()
        self.alarm_msg = String()
        self.fault_detected = False

        # Timer for periodic polling (3 Hz)
        self.timer = self.create_timer(0.1, self.read_and_publish)
        self.timer_poll = self.create_timer(1.0 / 3.0, self.publish_status)
        self.get_logger().info('[GreenwayBMSNode]: Initialized and started.')

    def read_and_publish(self):
        try:
            decoded_data = self.bms.readBMS()
            if decoded_data is None:
                return

            id_str = decoded_data.get('ID', "")
            if id_str == "0x0EA1F40D":
                self.battery_msg.voltage = float(decoded_data.get('Total Voltage (V)', 0.0))
                self.battery_msg.current = float(decoded_data.get('Total Current (A)', 0.0))

            elif id_str == "0x0EA0F40D":
                soc = float(decoded_data.get('SOC (%)', 0)) / 100.0
                soh = float(decoded_data.get('SOH (%)', 0))
                online_packs = decoded_data.get('Online Packs', 0)
                fault_status = decoded_data.get('Fault Status', "Normal")
                charging_status = decoded_data.get('Charging Status', "Not Charging")
                discharge_cycles = int(decoded_data.get('Discharge Cycles', 0))

                self.battery_msg.percentage = soc
                # Map SOH % to power_supply_health enum
                if soh >= 80:
                    self.battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
                else:
                    self.battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN

                self.battery_msg.present = bool(online_packs)

                if fault_status == "Normal":
                    self.fault_detected = False
                    if charging_status == "Charging":
                        self.battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
                    elif charging_status == "Not Charging":
                        self.battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                    else:
                        self.battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
                else:
                    self.get_logger().error("[GreenwayBMSNode]: Fault Detected")
                    self.fault_detected = True
                    self.battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN

                self.discharge_cycles.data = discharge_cycles

            elif id_str == "0x0EA2F40D":
                lowest_volt = float(decoded_data.get('Lowest Cell Voltage (V)', 0.0))
                highest_volt = float(decoded_data.get('Highest Cell Voltage (V)', 0.0))
                lowest_temp = float(decoded_data.get('Lowest Cell Temperature (°C)', 0.0))
                highest_temp = float(decoded_data.get('Highest Cell Temperature (°C)', 0.0))
                avg_temp = float(decoded_data.get('Average Cell Temperature (°C)', 0.0))

                self.battery_msg.cell_voltage = [lowest_volt, highest_volt]
                self.battery_msg.cell_temperature = [lowest_temp, highest_temp]
                self.battery_msg.temperature = avg_temp
                self.battery_temp_msg.temperature = avg_temp

            elif id_str == "0x0EA3F40D":
                if self.fault_detected:
                    self.alarm_msg.data = str(decoded_data)
                else:
                    self.alarm_msg.data = ""

            # Publish all
            # self.pub_battery_state.publish(self.battery_msg)
            # self.pub_battery_alarm.publish(self.alarm_msg)
            # self.pub_battery_cycle.publish(self.discharge_cycles)
            # self.pub_temperature.publish(self.battery_temp_msg)
            

        except KeyError as e:
            self.get_logger().warn(f"[GreenwayBMSNode]: Missing key in BMS data: {e}")
        except Exception as e:
            self.get_logger().error(f"[GreenwayBMSNode]: Exception during read/publish: {e}")

    def publish_status(self):
        self.pub_battery_state.publish(self.battery_msg)
        self.pub_battery_alarm.publish(self.alarm_msg)
        self.pub_battery_cycle.publish(self.discharge_cycles)
        self.pub_temperature.publish(self.battery_temp_msg)

    def shutdown(self):
        self.get_logger().info("Shutting down BMS node and CAN bus interface...")
        self.bms.shutDownBMS()

def main(args=None):
    rclpy.init(args=args)
    node = GreenwayBMSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

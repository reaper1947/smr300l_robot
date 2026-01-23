#!/usr/bin/env python3
import can

class GREENWAY_BMS():
    def __init__(self):
        self.can_device = str()
        self.read_timeout = float()
        self.bus = None  # Initialize bus object as None initially
    
    def initBMS(self, candevice_, read_timeout_):
        self.can_device = candevice_
        self.read_timeout = read_timeout_
        try:
            # Create a CAN bus object using the can0 interface
            self.bus = can.interface.Bus(channel=self.can_device, bustype='socketcan')
            print(f"[greenwayprotocal.initBMS]: Initial CANBUS {self.can_device} Success!!")
            return True
        except can.CanError as e:  # Specific exception for CAN errors
            print(f"[greenwayprotocal.initBMS]: Initial CANBUS {self.can_device} Fail!! Error: {e}")
            return False
    
    def shutDownBMS(self):
        print(f"[greenwayprotocal.shutDownBMS]: Shuttingdown CANBUS {self.can_device} ....")
        self.bus.shutdown()
    
    def readBMS(self):
        """
        This function reads a message from the CAN bus and decodes it based on its ID.
        It will print the decoded data for various IDs (BMS-related data).
        If no message is received within the timeout, it will return an error message.
        """
        try:
            # Receive a message from the bus with a timeout of 1.0 seconds
            message = self.bus.recv(self.read_timeout)  # Timeout based on user-defined value
            if message:
                # Decode the message based on its arbitration ID
                decoded_data = self.decode_bms_message(message)
                
                if 'error' in decoded_data:
                    # If the message ID is unknown, we print an error message
                    # print(f"Error: {decoded_data['error']}")
                    pass
                else:
                    # Print the decoded data
                    # print(f"[greenwayprotocal.readBMS]: Decoded Data: {decoded_data}")
                    return decoded_data
            else:
                print(f"[greenwayprotocal.readBMS]: No message received within {self.read_timeout} seconds.")
                return None
        
        except can.CanError as e:
            print(f"[greenwayprotocal.readBMS]: CAN Bus error while reading: {e}")
            return None
        except Exception as e:
            print(f"[greenwayprotocal.readBMS]: Unexpected error: {e}")
            return None

    
    def decode_bms_message(self, message):
        """
        Decode BMS CAN message based on ID and data structure.
        """
        if message.arbitration_id == 0x0EA0F40D:
            # Decode data for ID 0x0EA0F40D
            soc = message.data[0]  # Byte 1
            soh = message.data[1]  # Byte 2
            discharge_cycle = (message.data[2] << 8) | message.data[3]  # Byte 3-4
            online_packs = message.data[4]  # Byte 5
            operating_status = message.data[5]  # Byte 6
            charging_status = message.data[6]  # Byte 7
            fault_status = message.data[7]  # Byte 8

            return {
                "ID": "0x0EA0F40D",
                "SOC (%)": soc,
                "SOH (%)": soh,
                "Discharge Cycles": discharge_cycle,
                "Online Packs": online_packs,
                "Operating Status": "Standby" if operating_status == 0 else "Working",
                "Charging Status": "Not Charging" if charging_status == 0 else "Charging",
                "Fault Status": "Normal" if fault_status == 0 else "Fault Detected",
            }

        elif message.arbitration_id == 0x0EA1F40D:
            # Decode data for ID 0x0EA1F40D
            total_current = (
                (message.data[0] << 24)
                | (message.data[1] << 16)
                | (message.data[2] << 8)
                | message.data[3]
            )
            if total_current >= 0x80000000:
                total_current -= 0x100000000
            total_current = total_current / 1000.0  # Convert to A

            total_voltage = (
                (message.data[4] << 24)
                | (message.data[5] << 16)
                | (message.data[6] << 8)
                | message.data[7]
            )
            total_voltage = total_voltage / 1000.0  # Convert to V

            return {
                "ID": "0x0EA1F40D",
                "Total Current (A)": total_current,
                "Total Voltage (V)": total_voltage,
                "Total Watt (W)": abs(total_current*total_voltage)
            }

        elif message.arbitration_id == 0x0EA2F40D:
            # Decode data for ID 0x0EA2F40D
            lowest_temp = message.data[0] - 40  # Byte 1
            highest_temp = message.data[1] - 40  # Byte 2
            average_temp = message.data[2] - 40  # Byte 3

            lowest_cell_voltage = (
                (message.data[4] << 8) | message.data[5]
            ) / 1000.0  # Byte 5-6, Convert to V
            highest_cell_voltage = (
                (message.data[6] << 8) | message.data[7]
            ) / 1000.0  # Byte 7-8, Convert to V

            return {
                "ID": "0x0EA2F40D",
                "Lowest Cell Temperature (°C)": lowest_temp,
                "Highest Cell Temperature (°C)": highest_temp,
                "Average Cell Temperature (°C)": average_temp,
                "Lowest Cell Voltage (V)": lowest_cell_voltage,
                "Highest Cell Voltage (V)": highest_cell_voltage,
            }

        elif message.arbitration_id == 0x0EA3F40D:
            # Decode data for ID 0x0EA3F40D
            fault_flags_1 = message.data[0]  # Byte 1
            fault_flags_2 = message.data[1]  # Byte 2
            fault_flags_3 = message.data[2]  # Byte 3

            faults = {
                "ID": "0x0EA3F40D",
                "Primary Overvoltage": bool(fault_flags_1 & 0x01),
                "Secondary Overvoltage": bool(fault_flags_1 & 0x02),
                "Full Charge Protection": bool(fault_flags_1 & 0x04),
                "Charging Overcurrent": bool(fault_flags_1 & 0x08),
                "Charging High Temperature": bool(fault_flags_1 & 0x10),
                "Charging Low Temperature": bool(fault_flags_1 & 0x20),
                "Charging Timeout": bool(fault_flags_1 & 0x40),
                "Primary Undervoltage": bool(fault_flags_1 & 0x80),
                
                "Secondary Undervoltage": bool(fault_flags_2 & 0x01),
                "Primary Discharge Overcurrent": bool(fault_flags_2 & 0x02),
                "Secondary Discharge Overcurrent": bool(fault_flags_2 & 0x04),
                "Short Circuit": bool(fault_flags_2 & 0x08),
                "Discharge High Temperature": bool(fault_flags_2 & 0x10),
                "Discharge Low Temperature": bool(fault_flags_2 & 0x20),
                "MOS High Temperature Protection": bool(fault_flags_2 & 0x40),
                "Low Voltage Prohibits Charge/Discharge": bool(fault_flags_2 & 0x80),
                
                "Inter-group Cycle Difference Too Large": bool(fault_flags_3 & 0x01),
                "Cell Voltage Difference Too Large": bool(fault_flags_3 & 0x02),
                "Reserved": bool(fault_flags_3 & 0x04),  # Bits 2-7 reserved
            }

            return faults

        elif message.arbitration_id == 0x0EA4F40D:
            # Decode data for ID 0x0EA4F40D
            charge_voltage_high = (message.data[0] << 8) | message.data[1]  # Byte 1-2
            charge_current_high = (message.data[2] << 8) | message.data[3]  # Byte 3-4

            charge_voltage = charge_voltage_high / 100.0  # Convert to V
            charge_current = charge_current_high / 100.0  # Convert to A

            return {
                "ID": "0x0EA4F40D",
                "Allowed Charging Voltage (V)": charge_voltage,
                "Allowed Charging Current (A)": charge_current,
            }

        else:
            return {"error": "Unknown message ID"}

            
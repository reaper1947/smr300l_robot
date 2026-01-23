#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from next2_msgs.msg import LEDControl
from PyDMX.pydmx import PyDMX

class PyDMXROS(Node):
    def __init__(self):
        super().__init__('pydmx_ros')

        #====================================================================#
        #                 INIT GREENWAY_BMS Module connection                #
        #====================================================================#
        # self.port = self.declare_parameter("/ttyUSB4", None).value
        # self.port = serial.Serial(port='/dev/ttyACM0', baudrate=9600, parity='N', stopbits=1, bytesize=8, timeout=2.0)

        # self.dmx = PyDMX(self.port)
        self.dmx = PyDMX(COM='/dev/ttyUSB4', Cnumber=512)

        #====================================================================#
        #                      INIT ROS PARAMETER                            #
        #====================================================================#
        self.m_led_subscriber = self.create_subscription(
            LEDControl,
            "led_cmd",
            self.led_cmd_callback,
            10
        )

        self.color_code1 = dict()
        self.color_code2 = dict()
        self.control_flag = "BLINK"

        self.timer = self.create_timer(1.0 / 3.0, self.start)

    def led_cmd_callback(self, msg):
        # self.color_code1['rgb_hex'] = self.rgb_to_hex_safe(
        #     msg.leds.r, msg.leds.g, msg.leds.b
        # )
        # self.color_code2['rgb_hex'] = self.rgb_to_hex_safe(
        #     msg.leds2.r, msg.leds2.g, msg.leds2.b
        # )
        self.color_code1['rgb_hex'] = self.rgb_to_hex_safe(
            255, 0, 0
        )
        self.color_code2['rgb_hex'] = self.rgb_to_hex_safe(
            0, 255, 0
        )

        if self.control_flag == "BLINK":
            self.control_flag = "BLINK"
        elif self.control_flag == "SET":
            self.control_flag = "SET"
        else:
            self.control_flag = "OFF"

    @staticmethod
    def rgb_to_hex_safe(r, g, b):
        """
        Safely convert RGB values to HEX, ensuring valid range.
        :param r: Red (0-255)
        :param g: Green (0-255)
        :param b: Blue (0-255)
        :return: HEX color string
        """
        if not all(0 <= val <= 255 for val in (r, g, b)):
            raise ValueError("RGB values must be in the range 0-255")
        return f"#{r:02x}{g:02x}{b:02x}".upper()

    # def start(self):
    #     print(self.control_flag)
    #     if self.control_flag == "BLINK":
    #         self.dmx.fade_hex_time(
    #             self.color_code1['rgb_hex'],
    #             self.color_code2['rgb_hex'],
    #             duration=0.25
    #         )
    #         self.dmx.fade_hex_time(
    #             self.color_code2['rgb_hex'],
    #             self.color_code1['rgb_hex'],
    #             duration=0.5
    #         )
    #     elif self.control_flag == "SET":
    #         self.dmx.set_color(self.color_code1['rgb_hex'])
    #     else:
    #         self.dmx.set_color("#C8FF00")
    def start(self):
        print(self.control_flag)
        if self.control_flag == "BLINK":
            if 'rgb_hex' in self.color_code1 and 'rgb_hex' in self.color_code2:
                self.dmx.fade_hex_time(
                    self.color_code1['rgb_hex'],
                    self.color_code2['rgb_hex'],
                    duration=0.25
                )
                self.dmx.fade_hex_time(
                    self.color_code2['rgb_hex'],
                    self.color_code1['rgb_hex'],
                    duration=0.5
                )
        elif self.control_flag == "SET":
            if 'rgb_hex' in self.color_code1:
                self.dmx.set_color(self.color_code1['rgb_hex'])
        else:
            self.dmx.set_color("#08D334")



    def stop(self):
        self.dmx.__del__()

def main(args=None):
    rclpy.init(args=args)

    pydmx_ros = PyDMXROS()

    try:
        rclpy.spin(pydmx_ros)
    except KeyboardInterrupt:
        pass
    finally:
        pydmx_ros.stop()
        pydmx_ros.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import time
from PyDMX.pydmx import PyDMX


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

color_code1 = {'rgb_hex': rgb_to_hex_safe(0, 150, 0)}
color_code2 = {'rgb_hex': rgb_to_hex_safe(150, 0, 0)}
control_flag = "BLINK"

dmx = PyDMX(COM='/dev/ttyUSB4', Cnumber=512)
try:

    while 1000:
        if control_flag == "BLINK":
            dmx.fade_hex_time(
                color_code1['rgb_hex'],
                color_code2['rgb_hex'],
                duration=0.1
            )
            dmx.fade_hex_time(
                color_code2['rgb_hex'],
                color_code1['rgb_hex'],
                duration=0.1
            )
        if control_flag == "SET":
            if 'rgb_hex' in color_code1:
                dmx.set_color(color_code1['rgb_hex'])
        time.sleep(1)
except KeyboardInterrupt:
    print("Interrupted by user")
finally:
    del dmx

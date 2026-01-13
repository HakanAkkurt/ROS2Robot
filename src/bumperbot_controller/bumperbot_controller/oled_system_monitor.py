#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import psutil
from datetime import datetime

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106

from PIL import ImageFont

class OLEDSystemMonitor(Node):
    def __init__(self):
        super().__init__('oled_system_monitor')

        serial = i2c(port=1, address=0x3C)
        self.device = sh1106(serial, rotate=0)

        font_path = "/home/hakan/bitmap-fonts/bitmap/spleen/spleen-16x32.bdf"
        self.font = ImageFont.truetype(font_path, 32)

        # --- Timer Update (1 Hz) ---
        self.timer = self.create_timer(1.0, self.update_screen)

    def update_screen(self):
        now = datetime.now().strftime("%H:%M:%S")
        cpu = int(psutil.cpu_percent())
        # ram = int(psutil.virtual_memory().percent)

        with canvas(self.device) as draw:
            draw.text((0, 0),  f"{now}", font=self.font, fill=255)
            draw.text((0, 36), f"CPU:{cpu:<2}%", font=self.font, fill=255)
            # draw.text((0, 64), f"RAM:{ram:<2}%", font=self.font, fill=255)

def main(args=None):
    rclpy.init(args=args)
    node = OLEDSystemMonitor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

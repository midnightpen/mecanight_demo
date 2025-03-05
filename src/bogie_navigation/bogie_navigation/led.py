#! /usr/bin/env python3
#sudo -H pip install luma.core==1.13.0 luma.led_matrix==1.4.1

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
from luma.core.render import canvas
from luma.core.legacy import text, show_message
from luma.core.legacy.font import proportional, CP437_FONT, TINY_FONT, SINCLAIR_FONT, LCD_FONT
import time
led_show_time = 5

class led(Node):
    def __init__(self):
        super().__init__('led_display')
        self.subscription = self.create_subscription(String,'qr_data',self.led_callback,10)
        self.subscription  
        self.offset_x = 1
        self.offset_y = -2
        self.led_timer = self.create_timer(0.5,self.led_timers_callback)
        self.led_data = "UUUUUUUUUUUUUUUUU"

    def led_callback(self, msg):
        self.led_data = msg.data

    def led_timers_callback(self):
        global led_show_time
        text_to_display = self.led_data 
        # self.get_logger().info('I heard: "%s"' % text_to_display )
        if text_to_display == "xxx":
            serial = spi(port=1,device=0,gpio=noop())
            device = max7219(serial, width=32, height=8, block_orientation=-90, rotate=0)
            with canvas(device) as draw:
                text(draw, (4, 0), '', fill="white", font=proportional(TINY_FONT))
        else:
            serial = spi(port=1,device=0,gpio=noop())
            device = max7219(serial, width=32, height=8, block_orientation=-90, rotate=0)
            with canvas(device) as draw:
                text(draw, (4, 0), text_to_display, fill="white", font=proportional(TINY_FONT))
            time.sleep(led_show_time)


def main(args=None):
    rclpy.init(args=args)

    node= led()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
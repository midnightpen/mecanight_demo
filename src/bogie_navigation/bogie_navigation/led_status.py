import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import time
import gpio as GPIO

data_status = ""
led_show = 0

class led_status(Node):
    def __init__(self):
        super().__init__('cafe')
        self.sub_qr = self.create_subscription(String,'qr_data',self.status_callback,10)
        self.timer = self.create_timer(0.5,self.publish_status)
        self.get_logger().info('led status')
    def status_callback(self, msg):
        global data_status
        data_status = msg.data
    def publish_status(self):
        global data_status
        global led_show
        GPIO.setup(17, GPIO.OUT)

        if data_status == "0":
            GPIO.output(17, GPIO.HIGH)
        
        if data_status == "":
            GPIO.output(17, GPIO.LOW)
        
        if data_status == "1":
            if led_show == 0:
                led_show = 1
                GPIO.output(17, GPIO.HIGH)
            else:
                led_show = 0
                GPIO.output(17, GPIO.LOW)

def main(args=None):
    rclpy.init(args=args)
    node = led_status()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

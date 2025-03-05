import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
import cv2
import numpy as np
import time

class qrcode(Node):

    def __init__(self):
        super().__init__('qr_code')
        self.subscription = self.create_subscription(Image,'image',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()
        self.qr_code = String()
        self.pub_qr_data = self.create_publisher(String,'qr_data',10)
        self.timer = self.create_timer(0.5,self.timers_callback)

    def listener_callback(self, msg):
        try:
            cv_image = np.array(self.br.imgmsg_to_cv2(msg,desired_encoding="8UC3"))
            gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
            if len(decode(gray)) != 0:
                for code in decode(gray):
                    self.qr_code.data = code.data.decode("utf-8")
            else:
                self.qr_code.data = "xxx"
            #self.get_logger().info('I heard: "%s"' % self.qr_code.data )
        except CvBridgeError as e:
            print(e)

    def timers_callback(self):
        msg = String()
        msg.data = self.qr_code
        self.pub_qr_data.publish(msg.data)



def main(args=None):
    rclpy.init(args=args)
    node= qrcode()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
global qr_code
qr_code=''

class qrcode(Node):
    def __init__(self):
        
        super().__init__('cafe')
        client_cb_group = None
        timer_cb_group = None
        self.sub_qr = self.create_subscription(String,'qr_data',self.qr_callback,10,callback_group=client_cb_group)
        # self.pub_way = self.create_publisher(String,'my_way',10)
        self.timer = self.create_timer(0,self.publish_way)

        self.get_logger().info('Starting Navigation Commander')
        self.get_logger().info('Nav2 is activated')

        # self.publish_way()

    def qr_callback(self, msg):
        global qr_code
        qr_code = msg.data
    def publish_way(self):
        global qr_code
        print(qr_code)
        # while True:
        #     print(qr_code)
        #     continue




def main(args=None):
    global qr_code

    rclpy.init(args=args)
    node = qrcode()

    rclpy.spin(node)

    node.destroy_node(ft)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

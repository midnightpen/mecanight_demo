import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
import cv2 
import numpy as np
 
class ImagePublisher(Node):
  def __init__(self):
    super().__init__('image_publisher')  
    self.publisher_ = self.create_publisher(Image, 'image', 10)
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.cap = cv2.VideoCapture(0)
    self.br = CvBridge()
  

  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    ret, frame = self.cap.read()
    if ret == True:
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame, encoding='bgr8'))
  
def main(args=None):
  rclpy.init(args=args)
  image_publisher = ImagePublisher()
  rclpy.spin(image_publisher)
  image_publisher.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
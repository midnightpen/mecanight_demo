import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from bogie_interfaces.msg import SerialData    # CHANGE
from geometry_msgs.msg import Twist


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('test_sub')
        self.subscription = self.create_subscription(SerialData,'serial_data',self.listener_callback,10)
        self.subscription  #prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.1

    def listener_callback(self, msg):
        position_x = msg.pose.x
        position_y = msg.pose.y
        position_theta = msg.pose.theta
        #print(position_x,position_y,position_theta)
        if position_theta < 6.2831853: #angular: position_theta < 6.2831853, linear x: position_x < 1.0 backward position_x > -1.0, position_y > -1.0
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 1.0
            self.publisher_.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)         

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
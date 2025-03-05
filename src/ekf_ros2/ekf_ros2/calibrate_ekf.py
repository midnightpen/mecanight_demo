import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from bogie_interfaces.msg import SerialData    # CHANGE
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('calibrate_ekf')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.1

    def odom_callback(self, msg):
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        position_theta = theta[2]
        self.get_logger().info(f'Theta: {position_theta:.2f}')  # Log the current theta value
        if position_theta < 3.14 : #angular: position_theta < 6.2831853, linear x: position_x < 1.0 backward position_x > -1.0, position_y > -1.0
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.8
            # self.publisher_.publish(msg)
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
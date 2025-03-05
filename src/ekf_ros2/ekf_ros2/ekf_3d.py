import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import numpy as np

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom_ekf', 10)

        # Timer for prediction_and_update_step (running at 30Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.prediction_and_update_step)

        # Subscriptions to sensors
        self.odom_sub = self.create_subscription(Odometry, '/odom/raw', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(Odometry, '/odom_matcher', self.lidar_callback, 10)

        # Initialize state vector [x, y, theta]
        self.mu = np.zeros(3)
        self.mu_bar = np.zeros(3)

        # Encoder input (know input, v, w)
        self.u = np.zeros(2)

        # Initialize Observation input (x,y from lidar)
        self.z = np.zeros(2)

        # Covariance matrix
        self.sigma = np.array([ [0.1, 0, 0],
                                [0, 0.1, 0],
                                [0, 0, 0.1],
                              ])
        self.sigma_bar = np.zeros((3,3))

        # Process noise covariance (R)
        self.R = np.array([ [0.01, 0, 0],
                            [0, 0.01, 0],
                            [0, 0, 0.01],
                          ])

        # Observation noise covariance (Q) Lidar
        self.Q = np.array([ [0.02, 0],
                            [0, 0.02],
                          ])

        # Threshold for outlier detection
        self.outlier_threshold = 1.0  # ปรับค่า threshold ตามลักษณะการใช้งาน

        # Time variables
        self.last_time = self.get_clock().now()
    
    def is_outlier(self, innovation):
        """Function to check if the innovation is an outlier."""
        return np.linalg.norm(innovation) > self.outlier_threshold

    def publish_odom(self):

        odom_msg = Odometry()

        # Set the header information (frame and time)
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom_ekf'
        odom_msg.child_frame_id = 'base_footprint'

        # Set the pose (x, y, theta)
        odom_msg.pose.pose.position.x = self.mu[0]
        odom_msg.pose.pose.position.y = self.mu[1]
        odom_msg.pose.pose.position.z = 0.0


        # Convert theta (yaw) to quaternion for orientation
        quat = Quaternion()
        quat.z = np.sin(self.mu[2] / 2.0)
        quat.w = np.cos(self.mu[2] / 2.0)
        odom_msg.pose.pose.orientation = quat

        # Publish odometry
        self.odom_pub.publish(odom_msg)

    def odom_callback(self, msg):
        self.u[0] = msg.twist.twist.linear.x
        self.u[1] = msg.twist.twist.angular.z

        # self.z[0] = msg.pose.pose.position.x
        # self.z[1] = msg.pose.pose.position.y

    def lidar_callback(self, msg):
        self.z[0] = msg.pose.pose.position.x
        self.z[1] = msg.pose.pose.position.y

    def g(self, x, u, dt):
        """State transition function."""
        return np.array([
            x[0] + u[0] * np.cos(x[2]) * dt,
            x[1] + u[0] * np.sin(x[2]) * dt,
            x[2] + u[1] * dt
        ])

    def prediction_and_update_step(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Predict state estimate (mu)
        self.mu_bar = self.g(self.mu, self.u, dt)

        # The Jacobian matrix (G) of the nonlinear function (g)
        self.G = np.array([ [1, 0, -self.u[0] * np.sin(self.mu_bar[2]) * dt],
                            [0, 1, self.u[0] * np.cos(self.mu_bar[2]) * dt],
                            [0, 0, 1]
                         ])

        # Predict covariance (Sigma)
        self.sigma_bar = self.G @ self.sigma @ self.G.T + self.R

        # The nonlinear observation function (h)
        self.h = self.mu_bar

        # The Jacobian matrix (H) of the nonlinear function (h)
        self.H = np.array([ [1, 0, 0], 
                            [0, 1, 0]
                         ])

        # Compute the Kalman gain (K)
        K = self.sigma_bar @ self.H.T @ np.linalg.inv(self.H @ self.sigma_bar @ self.H.T + self.Q)

        # Calculate innovation (difference between measurement and prediction)
        innovation = self.z - self.h[:2]  # Use only x, y from h

        # Check if the measurement is an outlier
        if self.is_outlier(innovation):
            self.get_logger().warn('Outlier detected, skipping update step.')
            return

        # Update state estimate (mu)
        self.mu = self.mu_bar + K @ (self.z - self.h[:2]) # Use only the x, y from h

        # Update covariance (Sigma)
        I = np.eye(3)
        self.sigma = (I - K @ self.H) @ self.sigma_bar

        # Publish updated odometry
        self.publish_odom()

        self.get_logger().info(f'State: {self.mu}')

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

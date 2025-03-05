import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf_transformations import quaternion_from_euler, euler_from_quaternion
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
        self.imu_sub = self.create_subscription(Imu, '/Imu_data', self.imu_callback, 10)
        self.realsense_sub = self.create_subscription(Odometry, '/realsense_odom', self.realsense_callback, 10)

        # Initialize state vector [x, y, theta, w, ax, ay]
        self.mu = np.zeros(6)
        self.mu_bar = np.zeros(6)

        # Encoder input (know input, v, w)
        self.u = np.zeros(2)

        # Initialize Observation input ([lidar : x, y, || IMU: theta, w, ax, ay,])
        self.z = np.zeros(6)
        self.z_lidar = np.zeros(2)
        self.z_realsense = np.zeros(2)

        # Initialize Kalman Gain and innovation
        self.k = None
        self.innovation = None
        self.H = None

        # Covariance matrix
        self.sigma = np.eye(6)*0.1
        self.sigma_bar = np.zeros((6,6))

        # Process noise covariance (R)
        self.R = np.eye(6)*0.01

        # Observation noise covariance (Q) Lidar
        self.Q = np.array([ [0.02, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.02, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.01, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.01, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.01, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.01],
                          ])

        # Threshold for outlier detection
        self.outlier_threshold = 0.5  # ปรับค่า threshold ตามลักษณะการใช้งาน

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
        quat.x, quat.y, quat.z, quat.w = quaternion_from_euler(0.0, 0.0, self.mu[2])
        odom_msg.pose.pose.orientation = quat

        # Publish odometry
        self.odom_pub.publish(odom_msg)

    def odom_callback(self, msg):
        self.u[0] = msg.twist.twist.linear.x
        self.u[1] = msg.twist.twist.angular.z

    def lidar_callback(self, msg):
        self.z_lidar[0] = msg.pose.pose.position.x
        self.z_lidar[1] = msg.pose.pose.position.y

    def realsense_callback(self, msg):
        self.z_realsense[0] = msg.pose.pose.position.x
        self.z_realsense[1] = msg.pose.pose.position.y

    def imu_callback(self, msg):
        """Callback function for IMU data."""
        # Convert quaternion to euler to extract theta (yaw)
        # Convert quaternion to euler to extract theta (yaw)
        orientation_q = msg.orientation
        euler = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.z[2] = euler[2]
        self.z[3] = msg.angular_velocity.z
        self.z[4] = msg.linear_acceleration.x
        self.z[5] = msg.linear_acceleration.y

    def g(self, x, u, dt):
        """State transition function."""
        return np.array([
            x[0] + u[0] * np.cos(x[2]) * dt + 0.5 * x[4] * dt**2,   # x position update
            x[1] + u[0] * np.sin(x[2]) * dt + 0.5 * x[5] * dt**2,   # y position update
            x[2] + u[1] * dt,                                       # theta update (angular velocity)
            x[3],                                                   # angular velocity remains the same
            x[4],                                                   # acceleration in x remains the same
            x[5]                                                    # acceleration in y remains the same
        ])

    def prediction_and_update_step(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Predict state estimate (mu)
        self.mu_bar = self.g(self.mu, self.u, dt)

        # The Jacobian matrix (G) of the nonlinear function (g)
        self.G = np.array([ [1, 0, -self.u[0] * np.sin(self.mu_bar[2]) * dt, 0, 0.5 * dt**2, 0],
                            [0, 1, self.u[0] * np.cos(self.mu_bar[2]) * dt, 0, 0, 0.5 * dt**2],
                            [0, 0, 1, dt, 0, 0],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]
                         ])

        # Predict covariance (Sigma)
        self.sigma_bar = self.G @ self.sigma @ self.G.T + self.R

        # The nonlinear observation function (h)
        self.h = self.mu_bar

        # Calculate Kalman Gain for LiDAR
        H_lidar = np.array([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0]])
        K_lidar = self.sigma_bar @ H_lidar.T @ np.linalg.inv(H_lidar @ self.sigma_bar @ H_lidar.T + self.Q[:2, :2])

        # Calculate Kalman Gain for RealSense
        H_realsense = np.array([[1, 0, 0, 0, 0, 0],
                                [0, 1, 0, 0, 0, 0]])
        K_realsense = self.sigma_bar @ H_realsense.T @ np.linalg.inv(H_realsense @ self.sigma_bar @ H_realsense.T + self.Q[:2, :2])

        # คำนวณค่าความแปรปรวนจาก Kalman Gain
        cov_lidar = np.linalg.norm(H_lidar @ self.sigma_bar @ H_lidar.T + self.Q[:2, :2])
        cov_realsense = np.linalg.norm(H_realsense @ self.sigma_bar @ H_realsense.T + self.Q[:2, :2])

        if cov_lidar < cov_realsense:
            self.k = K_realsense
            self.H = H_realsense  # กำหนด H ให้เป็น H_realsense
            self.z[0] = self.z_realsense[0]
            self.z[1] = self.z_realsense[1]
        else:
            self.k = K_lidar
            self.H = H_lidar  # กำหนด H ให้เป็น H_lidar
            self.z[0] = self.z_lidar[0]
            self.z[1] = self.z_lidar[1]

        # Calculate innovation (difference between measurement and prediction)
        self.innovation = self.z - self.h  

        # Check if the measurement is an outlier
        if self.is_outlier(self.innovation):
            self.get_logger().warn('Outlier detected, skipping update step.')
            return

        # Update state estimate (mu)
        self.mu = self.mu_bar + self.k @ self.innovation

        # Update covariance (Sigma)
        I = np.eye(6)
        self.sigma = (I - self.k @ self.H) @ self.sigma_bar

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

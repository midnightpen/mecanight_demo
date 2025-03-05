import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer for prediction_and_update_step (running at 30Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.prediction_and_update_step)

        # Subscriptions to sensors
        self.odom_sub = self.create_subscription(Odometry, '/odom/raw', self.odom_callback, 10)
        # self.lidar_sub = self.create_subscription(Odometry, '/odom_lidar', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/Imu_data', self.imu_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize state vector [x, y, theta, vx, vy, w, ax, ay]
        self.mu_bar = np.zeros(8)
        self.mu = np.zeros(8)

        # Encoder input (know input, vx, vy, w)
        self.u = np.zeros(3)

        # Initialize Observation input ([Encoder : x, y, vx, vy || IMU: theta, w, ax, ay,])
        self.z = np.zeros(8)

        # Covariance matrix
        self.sigma = np.eye(8)*0.1
        self.sigma_bar = np.zeros((8,8))

        # Process noise covariance (R)
        self.R = np.eye(8)*0.01

        # Observation noise covariance (Q) Lidar
        self.Q = np.array([ [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # [0.02, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # [0.0, 0.02, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01],
                          ])

        # Threshold for outlier detection
        self.outlier_threshold = 20.0  # ปรับค่า threshold ตามลักษณะการใช้งาน

        # Time variables
        self.last_time = self.get_clock().now()

    
    def is_outlier(self, innovation):
        """Function to check if the innovation is an outlier."""
        mahalanobis_distance = np.sqrt(innovation.T @ np.linalg.inv(self.sigma_bar) @ innovation)
        return mahalanobis_distance > self.outlier_threshold

    def publish_odom(self):
        odom_frame_id = "odom"
        odom_child_frame_id = "base_footprint"

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = odom_frame_id
        t.child_frame_id = odom_child_frame_id
        t.transform.translation.x = self.mu[0]
        t.transform.translation.y = self.mu[1]
        t.transform.translation.z = 0.0

        # Convert theta (yaw) to quaternion for orientation
        q = quaternion_from_euler(0.0, 0.0, self.mu[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # Publishing odometry data
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = odom_frame_id
        odom_msg.child_frame_id = odom_child_frame_id

        odom_msg.pose.pose.position.x = self.mu[0]
        odom_msg.pose.pose.position.y = self.mu[1]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = t.transform.rotation
        odom_msg.twist.twist.linear.x = self.mu[3]
        odom_msg.twist.twist.linear.y = self.mu[4]
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.mu[5]

        # self.get_logger().info(f'Theta: {self.mu[2]:.2f}')  # Log the current theta value
        self.get_logger().info(f'x: {self.mu[0]:.2f}, y: {self.mu[1]:.2f}, theta: {self.mu[2]:.2f}')  # Log the current theta value

        self.odom_pub.publish(odom_msg)

    def odom_callback(self, msg):
        self.u[0] = msg.twist.twist.linear.x
        self.u[1] = msg.twist.twist.linear.y
        self.u[2] = msg.twist.twist.angular.z
        self.z[0] = msg.pose.pose.position.x
        self.z[1] = msg.pose.pose.position.y
        self.z[3] = msg.twist.twist.linear.x
        self.z[4] = msg.twist.twist.linear.y

    # def lidar_callback(self, msg):
    #     self.z[0] = msg.pose.pose.position.x
    #     self.z[1] = msg.pose.pose.position.y

    def imu_callback(self, msg):
        """Callback function for IMU data."""
        # Convert quaternion to euler to extract theta (yaw)
        orientation_q = msg.orientation
        euler = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

         # ตรวจสอบว่า self.initial_theta ถูกกำหนดแล้วหรือยัง
        if not hasattr(self, 'initial_theta'):
            self.initial_theta = euler[2]  # เก็บค่ามุม yaw เริ่มต้น

        self.z[2] = euler[2] - self.initial_theta
        # self.get_logger().info(f'theta: {self.z[2]:.2f}')  # Log the current theta value
        self.z[5] = msg.angular_velocity.z
        self.z[6] = msg.linear_acceleration.x
        self.z[7] = msg.linear_acceleration.y

    def g(self, x, u, dt):
        """State transition function."""
        return np.array([
            x[0] + ((u[0] * np.cos(x[2]) - u[1] * np.sin(x[2])) * dt) + 0.5 * x[6] * dt**2,     # x position update
            x[1] + ((u[0] * np.sin(x[2]) + u[1] * np.cos(x[2])) * dt) + 0.5 * x[7] * dt**2,     # y position update
            x[2] + u[2] * dt,                                                                   # theta update 
            x[3] + x[6] * dt,                                                                   # linear velocity in x remains the same
            x[4] + x[7] * dt,                                                                   # linear velocity in y remains the same
            x[5] + u[2],                                                                        # angular velocity 
            x[6],                                                                               # acceleration in x remains the same
            x[7]                                                                                # acceleration in y remains the same
        ])

    def jacobian_g(self, x, u, dt):
        """Jacobian g function."""
        return np.array([
            [1, 0, ((-u[0] * np.sin(x[2]) - u[1] * np.cos(x[2])) * dt), 0, 0, 0, (0.5 * dt**2), 0],
            [0, 1, ((u[0] * np.cos(x[2]) - u[1] * np.sin(x[2])) * dt),  0, 0, 0, 0, (0.5 * dt**2)],
            [0, 0, 1, 0, 0, 0, 0,  0],
            [0, 0, 0, 1, 0, 0, dt, 0],
            [0, 0, 0, 0, 1, 0, 0, dt],
            [0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1]                                                                            
        ])

    def prediction_and_update_step(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Predict state estimate (mu)
        self.mu_bar = self.g(self.mu, self.u, dt)

        # The Jacobian matrix (G) of the nonlinear function (g)
        # Update Jacobian G for mecanum
        self.G = self.jacobian_g(self.mu, self.u, dt)

        # Predict covariance (Sigma)
        self.sigma_bar = self.G @ self.sigma @ self.G.T + self.R

        # The nonlinear observation function (h)
        self.h = self.mu_bar

        # The Jacobian matrix (H) of the nonlinear function (h)
        self.H = np.eye(8)*1

        # Compute the Kalman gain (K)
        K = self.sigma_bar @ self.H.T @ np.linalg.inv(self.H @ self.sigma_bar @ self.H.T + self.Q)

        # Calculate innovation (difference between measurement and prediction)
        innovation = self.z - self.h  

        # Check if the measurement is an outlier
        if self.is_outlier(innovation):
            self.get_logger().warn('Outlier detected, skipping update step.')
            return

        # Update state estimate (mu)
        self.mu = self.mu_bar + K @ innovation

        # Update covariance (Sigma)
        I = np.eye(8)
        self.sigma = (I - K @ self.H) @ self.sigma_bar

        # self.get_logger().info(f'x: {self.mu[0]:.2f}, y: {self.mu[1]:.2f}, theta: {self.mu[2]:.2f}')  # Log the current theta value

        # Publish updated odometry
        self.publish_odom()

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

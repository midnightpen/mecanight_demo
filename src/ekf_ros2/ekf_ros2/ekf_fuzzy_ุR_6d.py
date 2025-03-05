# fuzzy logic adaptive Process Noise Covariance (R)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
from skfuzzy import control as ctrl  # Add this import for Fuzzy Logic
import skfuzzy as fuzz

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom_ekf', 10)

        # Timer for prediction_and_update_step (running at 30Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.prediction_and_update_step)

        # Subscriptions to sensors
        self.odom_sub = self.create_subscription(Odometry, '/odom/raw', self.odom_callback, 10)
        # self.lidar_sub = self.create_subscription(Odometry, '/odom_lidar', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/Imu_data', self.imu_callback, 10)

        # Initialize state vector [x, y, theta, w, ax, ay]
        self.mu = np.zeros(6)
        self.mu_bar = np.zeros(6)

        # Encoder input (know input, v, w)
        self.u = np.zeros(2)

        # Initialize Observation input ([lidar : x, y, || IMU: theta, w, ax, ay,])
        self.z = np.zeros(6)

        # Covariance matrix
        self.sigma = np.eye(6)*0.1
        self.sigma_bar = np.zeros((6,6))

        # Process noise covariance (R)
        self.R = np.eye(6)*0.01

        # Observation noise covariance (Q) Lidar
        self.Q = np.array([ [0.05, 0.0, 0.0, 0.0, 0.0, 0.0], # [0.02, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.05, 0.0, 0.0, 0.0, 0.0], # [0.0, 0.02, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.01, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.01, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.01, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.01],
                          ])

        # Threshold for outlier detection
        self.outlier_threshold = 4.0  # ปรับค่า threshold ตามลักษณะการใช้งาน

        # Time variables
        self.last_time = self.get_clock().now()

        # Initialize Fuzzy Logic for adjusting R
        self.init_fuzzy_logic()

    def init_fuzzy_logic(self):
        """Initialize fuzzy logic system for adjusting process noise covariance R."""
        # Define fuzzy variables for angular velocity and acceleration
        angular_velocity = ctrl.Antecedent(np.arange(-2, 2, 0.1), 'angular_velocity')
        acceleration = ctrl.Antecedent(np.arange(-10, 10, 0.5), 'acceleration')
        process_noise = ctrl.Consequent(np.arange(0.005, 0.05, 0.005), 'process_noise')

        # Define membership functions for inputs and output
        angular_velocity['low'] = fuzz.trimf(angular_velocity.universe, [-2, -1, 0])
        angular_velocity['medium'] = fuzz.trimf(angular_velocity.universe, [-1, 0, 1])
        angular_velocity['high'] = fuzz.trimf(angular_velocity.universe, [0, 1, 2])

        acceleration['low'] = fuzz.trimf(acceleration.universe, [-10, -5, 0])
        acceleration['medium'] = fuzz.trimf(acceleration.universe, [-5, 0, 5])
        acceleration['high'] = fuzz.trimf(acceleration.universe, [0, 5, 10])

        process_noise['low'] = fuzz.trimf(process_noise.universe, [0.005, 0.01, 0.015])
        process_noise['medium'] = fuzz.trimf(process_noise.universe, [0.01, 0.02, 0.03])
        process_noise['high'] = fuzz.trimf(process_noise.universe, [0.03, 0.04, 0.05])

        # Define rules
        rule1 = ctrl.Rule(angular_velocity['low'] & acceleration['low'], process_noise['low'])
        rule2 = ctrl.Rule(angular_velocity['medium'] & acceleration['medium'], process_noise['medium'])
        rule3 = ctrl.Rule(angular_velocity['high'] | acceleration['high'], process_noise['high'])

        # Build control system
        self.process_noise_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
        self.process_noise_sim = ctrl.ControlSystemSimulation(self.process_noise_ctrl)
    
    def adjust_process_noise_covariance(self):
        """Adjust process noise covariance R based on fuzzy logic."""
        # Input values from current IMU readings
        self.process_noise_sim.input['angular_velocity'] = self.mu[3]
        self.process_noise_sim.input['acceleration'] = np.linalg.norm([self.mu[4], self.mu[5]])
        
        # Compute fuzzy result and update R
        self.process_noise_sim.compute()
        adjusted_noise = self.process_noise_sim.output['process_noise']
        # Debug
        # print('adjusted_noise: %f' % (adjusted_noise))
        self.R = np.eye(6) * adjusted_noise
    
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
        self.z[0] = msg.pose.pose.position.x
        self.z[1] = msg.pose.pose.position.y

    # def lidar_callback(self, msg):
    #     self.z[0] = msg.pose.pose.position.x
    #     self.z[1] = msg.pose.pose.position.y

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

        # The Jacobian matrix (H) of the nonlinear function (h)
        self.H = np.eye(6)*1

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
        I = np.eye(6)
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

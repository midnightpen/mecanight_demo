import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from std_msgs.msg import String
from bogie_navigation.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
global qr_code

qr_code = ""

class qrcode(Node):

    def __init__(self):
        super().__init__('cafe')
        self.sub_qr = self.create_subscription(String,'qr_data',self.qr_callback,10)
        self.sub_qr  
        # self.pub_way = self.create_publisher(String,'my_way',10)
        # self.timer = self.create_timer(0.5,self.publish_way)

        self.get_logger().info('Starting Navigation Commander')
        self.nav= BasicNavigator()

        self.ip = PoseStamped()
        self.set_initial_pose()
        self.publish_way()
        self.wp = []


    def qr_callback(self, msg):
        global qr_code
        qr_code = msg.data
        print(qr_code)
        #self.qr_code.data = qr_code.data


    def publish_way(self):
        global qr_code
        self.nav.waitUntilNav2Active()
        self.get_logger().info('Nav2 is activated')

        target_1 = self.set_point(0.0,0.0,0.0)
        target_2 = self.set_point(1.989,0.009,0.0)
        target_3 = self.set_point(2.573,0.740,0.0)

        self.nav.goToPose(target_2)
        # while True:
        i = 0
        while not self.nav.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0:
                print('Distance remaining: ' + '{:.2f}'.format(
                    feedback.distance_remaining) + ' meters.')
                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.nav.cancelTask()
                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    self.nav.goToPose(target_2)
            #print(self.nav.isTaskComplete())
            # print("qr data")
            # print(qr_code)

        result = self.nav.getResult()

        print("go2")
        print(result)
        # print(qr_code)

        if result == TaskResult.SUCCEEDED :
            print('Goal succeeded!')
            print('Waiting QR')
            print(qr_code)
            while qr_code == "xxx":
                continue
            # time.sleep(3.5)
            self.nav.goToPose(target_1)
            j = 0
            while not self.nav.isTaskComplete():
                ################################################
                #
                # Implement some code here for your application!
                #
                ################################################

                # Do something with the feedback
                j = j + 1
                feedback = self.nav.getFeedback()
                if feedback and j % 5 == 0:
                    print('Distance remaining: ' + '{:.2f}'.format(
                        feedback.distance_remaining) + ' meters.')
                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        self.nav.cancelTask()

                    # Some navigation request change to demo preemption
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                        target_1.pose.position.x = -3.0
                        self.nav.goToPose(target_1)
            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
                exit(0)
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
                time.sleep(3.5)
                self.nav.goToPose(target_1)
                l = 0
                while not self.nav.isTaskComplete():
                    ################################################
                    #
                    # Implement some code here for your application!
                    #
                    ################################################

                    # Do something with the feedback
                    l = l + 1
                    feedback = self.nav.getFeedback()
                    if feedback and l % 5 == 0:
                        print('Distance remaining: ' + '{:.2f}'.format(
                            feedback.distance_remaining) + ' meters.')
                        # Some navigation timeout to demo cancellation
                        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                            self.nav.cancelTask()

                        # Some navigation request change to demo preemption
                        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                            target_1.pose.position.x = -3.0
                            self.nav.goToPose(target_1)
                if result == TaskResult.SUCCEEDED:
                    print('Goal succeeded!')
                    exit(0)
            elif result == TaskResult.FAILED:
                print('Goal failed!')
                self.nav.goToPose(target_1)
                p = 0
                while not self.nav.isTaskComplete():
                    ################################################
                    #
                    # Implement some code here for your application!
                    #
                    ################################################

                    # Do something with the feedback
                    p = p + 1
                    feedback = self.nav.getFeedback()
                    if feedback and p % 5 == 0:
                        print('Distance remaining: ' + '{:.2f}'.format(
                            feedback.distance_remaining) + ' meters.')
                        # Some navigation timeout to demo cancellation
                        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                            self.nav.cancelTask()

                        # Some navigation request change to demo preemption
                        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                            target_1.pose.position.x = -3.0
                            self.nav.goToPose(target_1)
                if result == TaskResult.SUCCEEDED:
                    print('Goal succeeded!')
                    exit(0)
        # if result == TaskResult.FAILED:
        #     continue


        # exit(0)

    def set_initial_pose(self):
        self.ip.header.frame_id = "map"
        self.ip.header.stamp = self.get_clock().now().to_msg()
        self.ip.pose.position.x = 0.00
        self.ip.pose.position.y = 0.00

        q = self.get_quaternion_from_euler(0,0,0)

        self.ip.pose.orientation.x = q[0]
        self.ip.pose.orientation.y = q[1]
        self.ip.pose.orientation.z = q[2]
        self.ip.pose.orientation.w = q[3]

        self.nav.setInitialPose(self.ip)
    
    def set_point(self,x,y,theta):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y

        q = self.get_quaternion_from_euler(0,0,theta)

        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        return goal_pose

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node= qrcode()
    # node.mission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

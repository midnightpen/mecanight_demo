#! /usr/bin/env python3

import rclpy 
import numpy as np
from rclpy.node import Node
from .robot_navigator import NavigationResult,BasicNavigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time

class navrobot(Node):
    def __init__(self):
        super().__init__('nav2_cmd_node')
        self.get_logger().info('Starting Navigation Commander')
        self.nav= BasicNavigator()
        self.nav.waitUntilNav2Active()
        self.get_logger().info('Nav2 is activated')

        self.ip = PoseStamped()
        self.set_initial_pose()
        self.wp = []
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
    def goto(self,x,y,theta):
        target = self.set_point(x,y,theta)
        self.nav.goToPose(target)

    def waypoint(self):
        p = self.set_point(2.33,-1.15,1.41)
        self.wp.append(p)
        p = self.set_point(0.96,-0.00,3.01)
        self.wp.append(p)
        p = self.set_point(1.54,-0.47,2.35)
        self.wp.append(p)

        #print(type(self.wp))

        self.nav.followWaypoints(self.wp)

        while not self.nav.isNavComplete():
            print(self.nav.isNavComplete())
            feedback =self.nav.getFeedback()
            print(feedback)
    

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]
def main(args=None):
    rclpy.init(args=args)
    nr =navrobot()
    nr.goto(3.98,-1.47,0.0)
    rclpy.spin(nr)
    nr.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()

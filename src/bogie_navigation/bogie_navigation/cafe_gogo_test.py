#!/usr/bin/python3
import multiprocessing as mp
import ctypes
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from bogie_navigation.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import numpy as np
import time


class my_cafe():
    def __init__(self,mp):
        self.cafeManager = mp.Manager()
        self.Buff = self.cafeManager.list()
        self.Buff_SSend = self.cafeManager.list()
        self.qr =  mp.Value('i',0)

def set_initial_pose(nav):
    navigation = nav
    ip = PoseStamped()
    ip.header.frame_id = "map"
    ip.header.stamp = navigation.get_clock().now().to_msg()
    ip.pose.position.x = 0.00
    ip.pose.position.y = 0.00

    q = get_quaternion_from_euler(0,0,0)

    ip.pose.orientation.x = q[0]
    ip.pose.orientation.y = q[1]
    ip.pose.orientation.z = q[2]
    ip.pose.orientation.w = q[3]

    nav.setInitialPose(ip)
    
def set_point(obj_cafe,x,y,theta):
    nav = BasicNavigator()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y

    q = get_quaternion_from_euler(0,0,theta)

    goal_pose.pose.orientation.x = q[0]
    goal_pose.pose.orientation.y = q[1]
    goal_pose.pose.orientation.z = q[2]
    goal_pose.pose.orientation.w = q[3]

    return goal_pose

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return [qx, qy, qz, qw]


class cafe_run(Node):
    def __init__(self,obj_cafe:my_cafe):
        #super().__init__('cafe')
        self.obj_cafe = obj_cafe
        #self.sub_qr = self.create_subscription(String,'qr_data',self.qr_callback,10)
        cafe_go()

    def cafe_go(obj_cafe): 
        while true:
            print(self.obj_cafe.qr.value)

def cafe_(obj_cafe):
    rclpy.init()
    nav = BasicNavigator()
    set_initial_pose(nav)
    go = cafe_run(obj_cafe)
    rclpy.spin(go)
    sub_QR.destroy_node()
    rclpy.shutdown() 

class SubQR_data(Node):
    def __init__(self,obj_cafe:my_cafe):
        super().__init__('cafe')
        self.obj_cafe = obj_cafe
        self.sub_qr = self.create_subscription(String,'qr_data',self.qr_callback,10)

    def qr_callback(self, msg):
        if msg.data == "xxx":
            self.obj_cafe.qr.value = 0
        else:
            self.obj_cafe.qr.value = 1         

def subQR(obj_cafe):
    rclpy.init()
    sub_QR = SubQR_data(obj_cafe)
    rclpy.spin(sub_QR)
    sub_QR.destroy_node()
    rclpy.shutdown()   


def main():
    try:
        obj_cafe = my_cafe(mp)   
        p_sub_qr  = mp.Process(target=subQR,args=(obj_cafe,))
        #p_runcafe = mp.Process(target=cafe_run,args=(obj_cafe,))
        p_sub_qr.start()
        #p_runcafe.start()
        while(1):
            1   
    except KeyboardInterrupt:
        p_sub_qr.kill()
        #p_runcafe.kill()

    finally:
        print("\n\nAll process is shutdown.")

if __name__ == '__main__':
    main()
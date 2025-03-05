#!/usr/bin/python3
import multiprocessing as mp
import ctypes
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import numpy as np
import time

class my_cafe():
    def __init__(self,mp):
        #self.cafeManager = mp.Manager()
        #self.Buff = self.cafeManager.list()
        #self.Buff_SSend = self.cafeManager.list()
        self.qr =  mp.Value('i',0)

def cafe_run(obj_cafe):     
    while(1):
        print(obj_cafe.qr.value)

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
        p_runcafe = mp.Process(target=cafe_run,args=(obj_cafe,))
        p_sub_qr.start()
        p_runcafe.start()
        while(1):
            1   
    except KeyboardInterrupt:
        p_sub_qr.kill()
        p_runcafe.kill()

    finally:
        print("\n\nAll process is shutdown.")

if __name__ == '__main__':
    main()
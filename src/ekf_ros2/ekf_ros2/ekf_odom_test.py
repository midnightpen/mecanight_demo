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
import numpy as np
import time

loop_mission = False

class my_cafe():
    def __init__(self,mp):
        #self.cafeManager = mp.Manager()
        #self.Buff = self.cafeManager.list()
        #self.Buff_SSend = self.cafeManager.list()
        self.qr =  mp.Value('i',0)

def set_point(x,y,theta):
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

def move_to_target(target_point):
    global nav
    while True:
        nav.goToPose(target_point)
        i = 0
        while not nav.isTaskComplete():
            i = i + 1
            feedback = nav.getFeedback()
            if feedback and i % 5 == 0:
                print('Distance remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED :
            break
        else:
            continue


def cafe_run(obj_cafe):
    global nav

    rclpy.init() 
    nav = BasicNavigator()

    # # #<<<<<<< Setup Targets Square>>>>>>>
    # target_1 = set_point(0.0,0.0,0.0)
    # target_2 = set_point(0.8,0.0,0.0)
    # target_3 = set_point(0.8,0.8,0.0)
    # target_4 = set_point(0.0,0.8,0.0)

    # # # <<<<<<< Setup Targets S>>>>>>>
    target_1 = set_point(0.0,0.0,0.0)
    target_2 = set_point(1.0,0.3,0.0)
    target_3 = set_point(2.0,-0.3,0.0)
    target_4 = set_point(2.5,0.3,0.0)
    target_5 = set_point(0.0,0.0,1.57079632679)

    #<<<<<<< InsitailPose >>>>>>>
    nav.setInitialPose(target_1)
    for i in range(5):
        while True:
        #<<<<<<< waitUntilNav2Active >>>>>>>
            nav.waitUntilNav2Active()
            nav.get_logger().info('Nav2 is activated')

        #<<<<<<< Mission >>>>>>>
        #<<<<<<< Start Target 1 >>>>>>>
            move_to_target(target_2)
            print('Goal 1 success')

        #<<<<<<< Start Target 2 >>>>>>> Go to table
            move_to_target(target_3)
            print('Goal 2 success') 

            move_to_target(target_4)
            print('Goal 3 success')

            move_to_target(target_5)
            print('Goal 4 success')

            i = i+1

            print('Round : %d' % i)

            break

    # #<<<<<<< Wait for QR (Order 1) >>>>>>> 
    #     while(1):
    #         if obj_cafe.qr.value == 0:
    #             print('Waiting QR')
    #         else:
    #             break

    # #<<<<<<< Start Target 3 >>>>>>> Go to prepair station
    #     move_to_target(target_3)
    #     print('Confirm to prepair station') 
    #     time.sleep(5)

    # #<<<<<<< Start Target 2 >>>>>>> Go to table
    #     move_to_target(target_2)
    #     print('Confirm to order table')

    # #<<<<<<< Wait for QR (Back home) >>>>>>> 
    #     while(1):
    #         if obj_cafe.qr.value == 2:
    #             break
    #         else:
    #             print('Waiting QR')
    #             continue

    # #<<<<<<< back to Start position >>>>>>>
    #     move_to_target(target_1)
    #     print('Mission complete')
        # if loop_mission:
        #     continue
        # else:
        #     break
            


# class SubQR_data(Node):
#     def __init__(self,obj_cafe:my_cafe):
#         super().__init__('cafe')
#         self.obj_cafe = obj_cafe
#         self.sub_qr = self.create_subscription(String,'qr_data',self.qr_callback,10)

#     def qr_callback(self, msg):
#         if msg.data == "xxx":
#             self.obj_cafe.qr.value = 0
#         elif msg.data == "end":
#             self.obj_cafe.qr.value = 2                
#         else:
#             self.obj_cafe.qr.value = 1         

# def subQR(obj_cafe):
#     rclpy.init()
#     sub_QR = SubQR_data(obj_cafe)
#     rclpy.spin(sub_QR)
#     sub_QR.destroy_node()
#     rclpy.shutdown()    

def main():
    try:
        obj_cafe = my_cafe(mp)   
        # p_sub_qr  = mp.Process(target=subQR,args=(obj_cafe,))
        p_runcafe = mp.Process(target=cafe_run,args=(obj_cafe,))
        # p_sub_qr.start()
        p_runcafe.start()
        while(1):
            1   
    except KeyboardInterrupt:
        # p_sub_qr.kill()
        p_runcafe.kill()

    finally:
        print("\n\nAll process is shutdown.")

if __name__ == '__main__':
    main()
#!/usr/bin/python3
import numpy as np
import serial
import multiprocessing as mp
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math 
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32,Float64,Int32
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from bogie_interfaces.msg import SerialData    # CHANGE

# Select Model
# 1 for 2 motor driver board protocal : Drivetrain - Differential drive 2 wheel
# 2 for 4 motor driver board protocal : Drivetrain - Differential drive 2 wheel
# 3 for 4 motor driver board protocal : Drivetrain - Differential drive 4 wheel (Skid Steer 4 wheel)
# 4 for 4 motor driver board protocal : Drivetrain - Mecanum      drive 4 wheel 
# 5 for 4 motor driver board protocal : Drivetrain - Omni         drive 3 wheel

models = 4
get_log = 1

class Uart():
    def __init__(self,mp):
        self.mana = mp.Manager()          ;  self.Buff = self.mana.list()
        self.flag_send =  mp.Value('i',0) ;  self.Buff_SSend = self.mana.list()
        self.port = "/dev/base_controller"          ;  self.baudrate = 38400
            
        # motor param
        self.speed_ratio = mp.Value('d',0.000062)   # unit: m/encode #0.0000415 #0.00045 #0.000608
        self.track_width = mp.Value('d',0.222)   # unit: m 0.29  #0.2
        self.wheel_base  = mp.Value('d',0.0522)   # unit: m 0.29  0.2

        self.encode_sampling_time = mp.Value('d',0.04)  # unit: s
        self.cmd_vel_linear_max_x = mp.Value('d',0.8)  # unit: m/s
        self.cmd_vel_linear_max_x = mp.Value('d',0.8)  # unit: m/s
        self.cmd_vel_linear_max_x = mp.Value('d',0.8)  # unit: m/s
        self.cmd_vel_linear_max_y = mp.Value('d',0.8)  # unit: m/s
        self.cmd_vel_angular_max = mp.Value('d',1.0)  #  unit: rad/s

        # odom result
        self.position_x = mp.Value('d',0.0)       # unit: m
        self.position_y = mp.Value('d',0.0)       # unit: m
        self.orientation = mp.Value('d',0.0)       # unit: rad
        self.velocity_linear_x = mp.Value('d',0.0)       # unit: m/s
        self.velocity_linear_y = mp.Value('d',0.0)       # unit: m/s
        self.velocity_linear_z = mp.Value('d',0.0)       # unit: m/s
        self.velocity_angular_x = mp.Value('d',0.0)       # unit: rad/s
        self.velocity_angular_y = mp.Value('d',0.0)       # unit: rad/s
        self.velocity_angular_z = mp.Value('d',0.0)       # unit: rad/s

        # speed result
        #self.wheel_speed_01 = mp.Value('f',0.0) ; self.wheel_speed_02 = mp.Value('f',0.0)
        #self.wheel_speed_03 = mp.Value('f',0.06 ; self.wheel_speed_04 = mp.Value('f',0.0)

        self.ser = self.check_port_open()
        self.target_1 = mp.Value('d',0.0)       ; self.target_2 = mp.Value('d',0.0)
        self.target_3 = mp.Value('d',0.0)       ; self.target_4 = mp.Value('d',0.0)

        self.feedback_1 = mp.Value('d',0.0)     ; self.feedback_2 = mp.Value('d',0.0)
        self.feedback_3 = mp.Value('d',0.0)     ; self.feedback_4 = mp.Value('d',0.0)

    def check_port_open(self):
        try:
            ser = serial.Serial(self.port,self.baudrate, timeout=1000 ,stopbits=1)
            print(self.port + ': port is Open.')
            return ser
        except:
            print(self.port + ': open port Fail.')
        return 0

def Receive_uart(Obj_uart): 
    print("Start Receive UART")
    header_check=0
    dada_amount = 0
    incoming_data = []

    global models
    select_model = models
    Protocal = 0
    if select_model == 1 :
        Protocal = 2
    else:
        Protocal = 4

    while(1):
        try:
            s = Obj_uart.ser.read(1)
            ss=int.from_bytes(s, byteorder="big",signed=0)
            # print(hex(ss))

            if(ss == 0XDC and header_check == 0): #DC
                header_check = 1
                incoming_data.append(ss)
            elif (ss == 0XBA and header_check == 1): #BA
                header_check = 2
                incoming_data.append(ss)
            elif(header_check ==2 ):
                if(dada_amount <= ((Protocal * 2) + 1 ) ):
                    incoming_data.append(ss)
                if(dada_amount == ((Protocal * 2) + 2 ) ): #if(dada_amount == ((Protocal * 2) + 2 ) ):
                    Obj_uart.Buff.append((incoming_data))
                    header_check = 0
                    dada_amount=0
                    incoming_data=[]
                dada_amount += 1
        except:
            Obj_uart.ser = Obj_uart.check_port_open()

def Transmit_uart(Obj_uart): 
    while(1):
        if(len(Obj_uart.Buff_SSend)>0):
            try:
                #print(Obj_uart.Buff_SSend)
                Obj_uart.ser.write(bytearray(Obj_uart.Buff_SSend[0]))
                Obj_uart.Buff_SSend.pop(0)
            except:
                Obj_uart.ser = Obj_uart.check_port_open()

def cal_uart(Obj_uart:Uart):
    global models
    select_model = models
    Protocal = 0
    if select_model == 1 :
        Protocal = 2
    else:
        Protocal = 4

    while(1):
        buffer = []
        if(len(Obj_uart.Buff)>0):
            buffer.extend(Obj_uart.Buff[0])
            Obj_uart.Buff.pop(0)
            if(len(buffer) >= ((Protocal * 2) + 2 )):
                while(1):
                    if(buffer[0] == 0xDC ) and (buffer[1] == 0xBA ):
                        checksum = sum(buffer[0:((Protocal * 2) + 2 )]) & 0XFF
                        if(checksum != buffer[-1]):
                            buffer =[]
                            # print("fail")
                            break
                        # print("good")

                        delta_encode_01_temp = 0 ; delta_encode_02_temp = 0
                        delta_encode_03_temp = 0 ; delta_encode_04_temp = 0

                        if select_model == 1 :
                            delta_encode_01_temp=buffer[2]<<8|buffer[3]
                            delta_encode_04_temp=buffer[4]<<8|buffer[5]
                            if (buffer[2] & 0X80)!=0:
                                delta_encode_01_temp=((delta_encode_01_temp^0XFFFF)+0X0001 )*-1
                            if (buffer[4] & 0X80)!=0:
                                delta_encode_04_temp=((delta_encode_04_temp^0XFFFF)+0X0001)*-1
                        else:
                            delta_encode_01_temp = buffer[2]<<8|buffer[3]
                            delta_encode_02_temp = buffer[4]<<8|buffer[5]
                            delta_encode_03_temp = buffer[6]<<8|buffer[7]
                            delta_encode_04_temp = buffer[8]<<8|buffer[9]
                            if (buffer[2] & 0X80)!=0:
                                delta_encode_01_temp = ((delta_encode_01_temp^0XFFFF)+0X0001)*-1
                            if (buffer[4] & 0X80)!=0:
                                delta_encode_02_temp = ((delta_encode_02_temp^0XFFFF)+0X0001)*-1
                            if (buffer[6] & 0X80)!=0:
                                delta_encode_03_temp = ((delta_encode_03_temp^0XFFFF)+0X0001)*-1
                            if (buffer[8] & 0X80)!=0:
                                delta_encode_04_temp = ((delta_encode_04_temp^0XFFFF)+0X0001)*-1
                   
                        Obj_uart.feedback_1.value =  delta_encode_01_temp #Pulse/0.04s
                        Obj_uart.feedback_2.value =  delta_encode_02_temp #Pulse/0.04s
                        Obj_uart.feedback_3.value =  delta_encode_03_temp #Pulse/0.04s
                        Obj_uart.feedback_4.value =  delta_encode_04_temp #Pulse/0.04s

                        dt = Obj_uart.encode_sampling_time.value
                        l1 = Obj_uart.track_width.value #/ 2
                        l2 = Obj_uart.wheel_base.value #/ 2
                        linear_x = 0.0   #m/s
                        linear_y = 0.0   #m/s
                        Angular_z = 0.0  #rad/s
                        delta_theta = 0.0 #rad

                        vel_1 =  delta_encode_01_temp * Obj_uart.speed_ratio.value / dt #m/s
                        vel_2 =  delta_encode_02_temp * Obj_uart.speed_ratio.value / dt #m/s
                        vel_3 =  delta_encode_03_temp * Obj_uart.speed_ratio.value / dt #m/s
                        vel_4 =  delta_encode_04_temp * Obj_uart.speed_ratio.value / dt #m/s

                        if select_model == 1 :
                            vel_L = vel_1 #m/s
                            vel_R = vel_4 #m/s
                            linear_x = (vel_R + vel_L)/2                            #m/s
                            linear_y = 0.0                                          #m/s
                            Angular_z = (vel_R - vel_L)/Obj_uart.track_width.value  #rad/s
                        elif select_model == 2 :
                            vel_L = vel_1 #m/s
                            vel_R = vel_4 #m/s
                            linear_x = (vel_R + vel_L)/2                            #m/s
                            linear_y = 0.0                                          #m/s
                            Angular_z = (vel_R - vel_L)/Obj_uart.track_width.value  #rad/s
                        elif select_model == 3 :
                            vel_L = (vel_1 + vel_2) / 2 #m/s
                            vel_R = (vel_3 + vel_4) / 2#m/s
                            linear_x = (vel_R + vel_L)/2                            #m/s
                            linear_y = 0.0                                          #m/s
                            Angular_z = (vel_R - vel_L)/Obj_uart.track_width.value  #rad/s
                        elif select_model == 4 :
                            linear_x  = ( vel_1 + vel_2 + vel_3 + vel_4) * (1/4)            #m/s
                            linear_y  = (-vel_1 + vel_2 + vel_3 - vel_4) * (1/4)            #m/s
                            Angular_z = (-vel_1 + vel_2 - vel_3 + vel_4) * (1/4) / (l1+l2)  #rad/s
                            

                        delta_theta = Angular_z * 0.04 # rad
                        Obj_uart.orientation.value = Obj_uart.orientation.value + delta_theta
                        orientation = Obj_uart.orientation.value 

                        delta_x = ((linear_x * math.cos(orientation)) - (linear_y * math.sin(orientation))) * dt
                        delta_y = ((linear_x * math.sin(orientation)) + (linear_y * math.cos(orientation))) * dt

                        Obj_uart.position_x.value = Obj_uart.position_x.value + delta_x
                        Obj_uart.position_y.value = Obj_uart.position_y.value + delta_y
                        Obj_uart.velocity_linear_x.value  = linear_x
                        Obj_uart.velocity_linear_y.value  = linear_y
                        Obj_uart.velocity_angular_z.value = Angular_z

                        #vx =  ( speed1 + speed2 + speed3 + speed4) * (R/4)
                        #vy =  (-speed1 + speed2 - speed3 + speed4) * (R/4)
                        #wz =  (-speed1 - speed2 + speed3 + speed4) * (R/4) / (l1+l2)
                        break

                    else:
                        print("Wrong data header (",buffer[0],"!= DC ",buffer[1],"!= BA ")
                        buffer =[]
                        break

def get_quaternion_from_euler(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

class Pubodom(Node):
    def __init__(self,Obj_uart:Uart):
        super().__init__('odom_publisher')
        self.Obj_uart = Obj_uart
        self.odom_pub_ = self.create_publisher(Odometry, 'odom/raw', 20)
        self.serail_data_pub = self.create_publisher(SerialData, 'serial_data', 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.cal_odom)
 
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.sub_cmd_cal,20)
        self.flag_time =False
        self.timer_ = self.create_timer(0.1,self.timer_callback_vel)
        self.timer_pre = time.time()

    def cal_odom(self):
        global get_log
        if get_log == 1 :
            self.get_logger().info("target1 = %d feedback1 = %d target2 = %d feedback2 = %d target3 = %d feedback3 = %d target4 = %d feedback4 = %d" % (self.Obj_uart.target_1.value, self.Obj_uart.feedback_1.value,self.Obj_uart.target_2.value, self.Obj_uart.feedback_2.value, self.Obj_uart.target_3.value, self.Obj_uart.feedback_3.value,self.Obj_uart.target_4.value, self.Obj_uart.feedback_4.value))
        odom_frame_id = "odom"
        odom_child_frame_id = "base_footprint"

        t=TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = odom_frame_id
        t.child_frame_id = odom_child_frame_id
        t.transform.translation.x = self.Obj_uart.position_x.value
        t.transform.translation.y = self.Obj_uart.position_y.value
        t.transform.translation.z = 0.0
        q=get_quaternion_from_euler(0.0,0.0,self.Obj_uart.orientation.value)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        # self.tf_broadcaster.sendTransform(t) # Open when have not imu

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = odom_frame_id
        odom.child_frame_id = odom_child_frame_id
        odom.pose.pose.position.x = self.Obj_uart.position_x.value
        odom.pose.pose.position.y = self.Obj_uart.position_y.value
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = self.Obj_uart.velocity_linear_x.value
        odom.twist.twist.linear.y = self.Obj_uart.velocity_linear_y.value
        odom.twist.twist.linear.z = self.Obj_uart.velocity_linear_z.value
        odom.twist.twist.angular.x = self.Obj_uart.velocity_angular_x.value
        odom.twist.twist.angular.y = self.Obj_uart.velocity_angular_y.value
        odom.twist.twist.angular.z = self.Obj_uart.velocity_angular_z.value

        covariance = [0.01, 0, 0, 0, 0, 0,  # covariance on gps_x
                        0, 0.01, 0, 0, 0, 0,  # covariance on gps_y
                        0, 0, 99999, 0, 0, 0, # covariance on gps_z
                        0, 0, 0, 99999, 0, 0, # large covariance on rot x
                        0, 0, 0, 0, 99999, 0, # large covariance on rot y
                        0, 0, 0, 0, 0, 0.01] # large covariance on rot z

        odom.pose.covariance = covariance

        self.odom_pub_.publish(odom)

        msg = SerialData()
        msg.target.target_1 = int(self.Obj_uart.target_1.value)      ; msg.target.target_2 = int(self.Obj_uart.target_2.value)
        msg.target.target_3 = int(self.Obj_uart.target_3.value)      ; msg.target.target_4 = int(self.Obj_uart.target_4.value)
        msg.feedback.feedback_1 = int(self.Obj_uart.feedback_1.value); msg.feedback.feedback_2 = int(self.Obj_uart.feedback_2.value)
        msg.feedback.feedback_3 = int(self.Obj_uart.feedback_3.value); msg.feedback.feedback_4 = int(self.Obj_uart.feedback_4.value)
        msg.pose.x = self.Obj_uart.position_x.value
        msg.pose.y = self.Obj_uart.position_y.value
        msg.pose.theta = self.Obj_uart.orientation.value
        msg.velocity.vx = self.Obj_uart.velocity_linear_x.value
        msg.velocity.vy = self.Obj_uart.velocity_linear_y.value       
        msg.velocity.vtheta = self.Obj_uart.velocity_angular_z.value
        
        self.serail_data_pub.publish(msg)
       

    def timer_callback_vel(self):
        if(self.flag_time and time.time()-self.timer_pre>=0.5):
            #print("[Send to Serial]: Stop")
            global models
            select_model = models
            Protocal = 0
            if select_model == 1 :
                write_buff = [0xab, 0xcd, 0x00, 0x00, 0x00, 0x00, 0x78]
                self.Obj_uart.Buff_SSend.append(write_buff)
                self.flag_time = False
            else:
                write_buff = [0xab, 0xcd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78]
                self.Obj_uart.Buff_SSend.append(write_buff)
                self.flag_time = False


    def sub_cmd_cal(self,msg):
        angular_temp = np.clip(msg.angular.z, -self.Obj_uart.cmd_vel_angular_max.value, self.Obj_uart.cmd_vel_angular_max.value)
        linear_temp_x = np.clip(msg.linear.x,  -self.Obj_uart.cmd_vel_linear_max_x.value,  self.Obj_uart.cmd_vel_linear_max_x.value)
        linear_temp_y = np.clip(msg.linear.y,  -self.Obj_uart.cmd_vel_linear_max_y.value,  self.Obj_uart.cmd_vel_linear_max_y.value)

        global models
        select_model = models
        delta_encode_temp_01 = 0 ; delta_encode_temp_02 = 0 ; delta_encode_temp_03 = 0 ; delta_encode_temp_04 = 0

        if select_model == 1 :
            delta_encode_left_temp = (linear_temp_x - 0.5 * (self.Obj_uart.track_width.value) * angular_temp) * (self.Obj_uart.encode_sampling_time.value) / (self.Obj_uart.speed_ratio.value)
            delta_encode_right_temp = (linear_temp_x + 0.5 * (self.Obj_uart.track_width.value) * angular_temp) * (self.Obj_uart.encode_sampling_time.value) / (self.Obj_uart.speed_ratio.value)
            self.Obj_uart.target_1.value = delta_encode_left_temp
            self.Obj_uart.target_2.value = 0
            self.Obj_uart.target_3.value = 0
            self.Obj_uart.target_4.value = delta_encode_right_temp
            delta_encode_temp_01 = delta_encode_left_temp
            delta_encode_temp_02 = delta_encode_right_temp
        
        elif select_model == 2 :
            delta_encode_left_temp = (linear_temp_x - 0.5 * (self.Obj_uart.track_width.value) * angular_temp) * (self.Obj_uart.encode_sampling_time.value) / (self.Obj_uart.speed_ratio.value)
            delta_encode_right_temp = (linear_temp_x + 0.5 * (self.Obj_uart.track_width.value) * angular_temp) * (self.Obj_uart.encode_sampling_time.value) / (self.Obj_uart.speed_ratio.value)
            self.Obj_uart.target_1.value = delta_encode_left_temp
            self.Obj_uart.target_2.value = 0
            self.Obj_uart.target_3.value = 0
            self.Obj_uart.target_4.value = delta_encode_right_temp
            delta_encode_temp_01 = delta_encode_left_temp
            delta_encode_temp_02 = 0
            delta_encode_temp_03 = 0
            delta_encode_temp_04 = delta_encode_right_temp
        elif select_model == 3 :
            delta_encode_left_temp = (linear_temp_x - 0.5 * (self.Obj_uart.track_width.value) * angular_temp) * (self.Obj_uart.encode_sampling_time.value) / (self.Obj_uart.speed_ratio.value)
            delta_encode_right_temp = (linear_temp_x + 0.5 * (self.Obj_uart.track_width.value) * angular_temp) * (self.Obj_uart.encode_sampling_time.value) / (self.Obj_uart.speed_ratio.value)
            self.Obj_uart.target_1.value = delta_encode_left_temp
            self.Obj_uart.target_2.value = delta_encode_left_temp
            self.Obj_uart.target_3.value = delta_encode_right_temp
            self.Obj_uart.target_4.value = delta_encode_right_temp
            delta_encode_temp_01 = delta_encode_left_temp
            delta_encode_temp_02 = delta_encode_left_temp
            delta_encode_temp_03 = delta_encode_right_temp
            delta_encode_temp_04 = delta_encode_right_temp
        elif select_model == 4 :
            l1 = self.Obj_uart.track_width.value
            l2 = self.Obj_uart.wheel_base.value

            delta_encode_1 = (linear_temp_x - linear_temp_y - ((l1+l2) * angular_temp)) * (self.Obj_uart.encode_sampling_time.value) / (self.Obj_uart.speed_ratio.value)
            delta_encode_2 = (linear_temp_x + linear_temp_y + ((l1+l2) * angular_temp)) * (self.Obj_uart.encode_sampling_time.value) / (self.Obj_uart.speed_ratio.value)
            delta_encode_3 = (linear_temp_x + linear_temp_y - ((l1+l2) * angular_temp)) * (self.Obj_uart.encode_sampling_time.value) / (self.Obj_uart.speed_ratio.value)
            delta_encode_4 = (linear_temp_x - linear_temp_y + ((l1+l2) * angular_temp)) * (self.Obj_uart.encode_sampling_time.value) / (self.Obj_uart.speed_ratio.value)

            self.Obj_uart.target_1.value = delta_encode_1
            self.Obj_uart.target_2.value = delta_encode_2
            self.Obj_uart.target_3.value = delta_encode_3
            self.Obj_uart.target_4.value = delta_encode_4
            delta_encode_temp_01 = delta_encode_1
            delta_encode_temp_02 = delta_encode_2
            delta_encode_temp_03 = delta_encode_3
            delta_encode_temp_04 = delta_encode_4
       

        if select_model == 1 :
            write_buff=[0,0,0,0,0,0,0]
            write_buff[0]= 0xAB
            write_buff[1]= 0xCD

            if(delta_encode_left_temp <0):
                delta_encode_left_temp=((int(abs(delta_encode_left_temp))^0XFFFF)+0X0001 )& 0XFFFF
                
            write_buff[2]= int(abs(delta_encode_left_temp)) >> 8  & 0xFF
            write_buff[3]= int(abs(delta_encode_left_temp)) & 0xFF

            if(delta_encode_right_temp <0):
                delta_encode_right_temp=((int(abs(delta_encode_right_temp))^0XFFFF)+0X0001 )& 0XFFFF

            write_buff[4]= int(abs(delta_encode_right_temp)) >> 8  & 0xFF
            write_buff[5]= int(abs(delta_encode_right_temp)) & 0xFF

            write_buff[6] = (write_buff[0] + write_buff[1] + write_buff[2] + write_buff[3] + write_buff[4]+ write_buff[5]) & 0xFF
            self.Obj_uart.Buff_SSend.append(write_buff)
            self.timer_pre = time.time()
            self.flag_time = True
        else:
            write_buff=[0,0,0,0,0,0,0,0,0,0,0]
            write_buff[0]= 0xAB
            write_buff[1]= 0xCD

            if(delta_encode_temp_01 <0):
                delta_encode_temp_01 =((int(abs(delta_encode_temp_01))^0XFFFF)+0X0001 )& 0XFFFF  
            write_buff[2]= int(abs(delta_encode_temp_01)) >> 8  & 0xFF
            write_buff[3]= int(abs(delta_encode_temp_01)) & 0xFF
    
            if(delta_encode_temp_02 <0):
                delta_encode_temp_02=((int(abs(delta_encode_temp_02))^0XFFFF)+0X0001 )& 0XFFFF
            write_buff[4]= int(abs(delta_encode_temp_02)) >> 8  & 0xFF
            write_buff[5]= int(abs(delta_encode_temp_02)) & 0xFF
    
            if(delta_encode_temp_03 <0):
                delta_encode_temp_03=((int(abs(delta_encode_temp_03))^0XFFFF)+0X0001 )& 0XFFFF
            write_buff[6]= int(abs(delta_encode_temp_03)) >> 8  & 0xFF
            write_buff[7]= int(abs(delta_encode_temp_03)) & 0xFF

            if(delta_encode_temp_04 <0):
                delta_encode_temp_04=((int(abs(delta_encode_temp_04))^0XFFFF)+0X0001 )& 0XFFFF
            write_buff[8]= int(abs(delta_encode_temp_04)) >> 8  & 0xFF
            write_buff[9]= int(abs(delta_encode_temp_04)) & 0xFF

            write_buff[10] = (write_buff[0] + write_buff[1] + write_buff[2] + write_buff[3] + write_buff[4] + write_buff[5] + write_buff[6] + write_buff[7] + write_buff[8]+ write_buff[9]) & 0XFF

            self.Obj_uart.Buff_SSend.append(write_buff)
            #print(hex(write_buff[0]),hex(write_buff[1]),hex(write_buff[2]),hex(write_buff[3]),hex(write_buff[4]),hex(write_buff[5]),hex(write_buff[6]),hex(write_buff[7]),hex(write_buff[8]),hex(write_buff[9]))
            self.timer_pre = time.time()
            self.flag_time = True

def pub(Obj_uart):
    rclpy.init()
    pub_odom = Pubodom(Obj_uart)
    rclpy.spin(pub_odom)
    pub_odom.destroy_node()
    rclpy.shutdown()

def main():
    try:
        Obj_uart = Uart(mp)   

        if(Obj_uart.ser!=0):
            p1  = mp.Process(target=Receive_uart,args=(Obj_uart,))
            p2 = mp.Process(target=Transmit_uart,args=(Obj_uart,))
            p3 = mp.Process(target=cal_uart,args=(Obj_uart,))
            p4 = mp.Process(target=pub,args=(Obj_uart,))

            p1.start() ; p2.start()
            p3.start() ; p4.start()
            while(1):
                1
        
    except KeyboardInterrupt:
        if(Obj_uart.ser!=0):
            p1.kill() ; p2.kill()
            p3.kill() ; p4.kill()

    finally:
        print("\n\nAll process is shutdown.")



if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import sys

import geometry_msgs.msg
import rclpy
import threading
import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

MAX_LIN_VEL = 0.5
MAX_ANG_VEL = 0.40

#MAX_LIN_VEL = rospy.get_param('~linear_speed_limit', 1.0)
#MAX_ANG_VEL = rospy.get_param('~angular_speed_limit', 5.0)

LIN_VEL_STEP_SIZE = 0.01 #default 0.01
ANG_VEL_STEP_SIZE = 0.05 #default 0.05

msg = """
Control Your Robot
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop
CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input


def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
    return vel


def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)
    return vel



def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    # parameters
    stamped = node.declare_parameter('stamped', False).value
    frame_id = node.declare_parameter('frame_id', '').value
    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)
    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    pv_target_linear_vel = 0.0
    pv_target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 0.0
    turn = 0.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    twist_msg = TwistMsg()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    try:
        print(msg)
        print(vels(speed, turn))
        pv_key = getKey(settings)

        while True:
            
            key = getKey(settings)
            if key == 'w':
                target_linear_vel = checkLinearLimitVelocity(
                    target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print (vels(target_linear_vel, target_angular_vel))
            elif key == 'x':
                target_linear_vel = checkLinearLimitVelocity(
                    target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print (vels(target_linear_vel, target_angular_vel))
            elif key == 'a':
                target_angular_vel = checkAngularLimitVelocity(
                    target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print (vels(target_linear_vel, target_angular_vel))
            elif key == 'd':
                target_angular_vel = checkAngularLimitVelocity(
                    target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print (vels(target_linear_vel, target_angular_vel))
            elif key == 's' or key == ' ' :
                target_linear_vel = 0.0
                control_linear_vel = 0.0
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                print (vels(target_linear_vel, target_angular_vel))
            else:
                if (key == '\x03'):
                    break
            #print (key)
            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            #control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/4.0))
  
            twist.linear.x = target_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            #control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/4.0))
 
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = target_angular_vel
            pub.publish(twist_msg)

    except Exception as e:
        print(e)

    finally:
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()

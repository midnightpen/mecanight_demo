#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from bogie_navigation.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import numpy as np

"""
Basic navigation demo to go to pose.
"""
def get_quaternion_from_euler(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    q = get_quaternion_from_euler(0.0,0.0,0.0)
    initial_pose.pose.orientation.x = q[0]
    initial_pose.pose.orientation.y = q[1]
    initial_pose.pose.orientation.z = q[2]
    initial_pose.pose.orientation.w = q[3]
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.70
    goal_pose.pose.position.y = 0.39
    q = get_quaternion_from_euler(0.0,0.0,0.0)
    goal_pose.pose.orientation.x = q[0]
    goal_pose.pose.orientation.y = q[1]
    goal_pose.pose.orientation.z = q[2]
    goal_pose.pose.orientation.w = q[3]

    goal_pose_1 = PoseStamped()
    goal_pose_1.header.frame_id = 'map'
    goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_1.pose.position.x = 0.0
    goal_pose_1.pose.position.y = 0.0
    q = get_quaternion_from_euler(0.0,0.0,1.57079632679)
    goal_pose_1.pose.orientation.x = q[0]
    goal_pose_1.pose.orientation.y = q[1]
    goal_pose_1.pose.orientation.z = q[2]
    goal_pose_1.pose.orientation.w = q[3]

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Distance remaining: ' + '{:.2f}'.format(
                  feedback.distance_remaining) + ' meters.')
            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                goal_pose.pose.position.x = -3.0
                navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.goToPose(goal_pose_1)
        j = 0
        while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
            j = j + 1
            feedback = navigator.getFeedback()
            if feedback and j % 5 == 0:
                print('Distance remaining: ' + '{:.2f}'.format(
                    feedback.distance_remaining) + ' meters.')
                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    goal_pose.pose.position.x = -3.0
                    navigator.goToPose(goal_pose)

        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    #navigator.lifecycleShutdown()

    #exit(0)


if __name__ == '__main__':
    main()
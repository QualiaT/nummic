#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np

from std_msgs.msg import Int8
from nav_msgs.msg import Odometry, Path

def calc_nav_to_target_distance():

    global nav_to_target_distance
    global path_offset
    global path_step_meter
    global mobile_robot_width
    global half_of_width

    ####################################################################################
    # distance from ground to manipulator's first_joint_height                         #
    a = rospy.get_param("/setting/first_joint_height")                                 #
    # target_z_coordinate                                                              #
    b = target_z                                                                       #
    # distance from mobile_base_lilnk to manipulator_base_link                         #
    c = rospy.get_param("/setting/between_base_link")                                  #
    # manipulator safty zone distance                                                  #
    d = rospy.get_param("/setting/recommended_reach")                                  #
    # navigation path step size                                                        #
    path_step_meter = rospy.get_param("/setting/granularity")                          #
    # mobile_robot_width                                                               #
    mobile_robot_width = rospy.get_param("/setting/mobile_robot_width")                #
    ####################################################################################

    if a - b > 0:
        target_theta = math.acos((a-b)/d)
    else:
        target_theta = math.acos((b-a)/d)
    
    nav_to_target_distance = d * math.sin(target_theta) + c
    
    path_offset_orig = round(nav_to_target_distance / path_step_meter)
    path_offset = int(path_offset_orig)
    half_of_width = mobile_robot_width / 2

def get_current_pose(msg):

    global current_x
    global current_y

    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    
    dx = target_x - current_x
    dy = target_y - current_y

    global distance2goal 
    distance2goal = math.sqrt(dx*dx + dy*dy)
    
    calc_current_pose_on_path()
    

def get_path_data(path_msg):

    global path_length
    path_length = 0
    
    global path_step
    path_step = len(path_msg.poses) - 1
    
    global path
    path = [[0.0, 0.0] for i in range(0)]
    
    global path_final_check
    
    for i in range(len(path_msg.poses) - 1):
        position_a_x = path_msg.poses[i].pose.position.x
        position_b_x = path_msg.poses[i+1].pose.position.x
        position_a_y = path_msg.poses[i].pose.position.y
        position_b_y = path_msg.poses[i+1].pose.position.y

        path_length += np.sqrt(np.power((position_b_x - position_a_x), 2) + np.power((position_b_y - position_a_y), 2))
    
    for i in range(path_step):
        position_x = path_msg.poses[i].pose.position.x
        position_y = path_msg.poses[i].pose.position.y
        path.append([position_x, position_y])
    
    
    path_curvature()
    path_curvature_check()

    
def path_curvature():

    global path
    global path_offset
    
    global curvature
    curvature = [0.0 for i in range(0)]

    global path_step

    global curvature_check
    curvature_check = [0.0 for i in range(0)]

    for i in range(path_step - 1):
        yaw_x = path[i+1][0] - path[i][0]
        yaw_y = path[i+1][1] - path[i][1]
        
        path_step_curvature = math.degrees(math.atan2(yaw_y, yaw_x))
        
        if path_step_curvature < 0:
            path_step_curvature = 360 + path_step_curvature
        
        curvature.append(path_step_curvature)
    
    for i in range(path_step - 2):
        check = curvature[i+1] - curvature[i]
        
        if check < -180:
            check = (360 + check)
                
        if check > 180:
            check = (360 - check)
            
        if check < 0:
            check = check * (-1)

        if i > (path_step - 2 - path_offset) or i == (path_step - 2 - path_offset):
            check = 0.0
        
        curvature_check.append(check)


def path_curvature_check():
    
    global path_final_check
    path_final_check = [0.0 for i in range(0)]
    
    global curvature_check
    global manipulation_safe_point
    
    for i in range(path_step - 2):
        
        if math.radians(curvature_check[i] == 0):
            Radius_of_curvature = 0
        else:
            Radius_of_curvature = path_step_meter / math.radians(curvature_check[i])
        
        
        # calculate the offset between manipulator's reach and path curvature safety area
        if Radius_of_curvature == 0:
            path_final_check.append(0)
        
        elif np.sqrt(np.power((Radius_of_curvature + half_of_width), 2) - np.power((Radius_of_curvature), 2)) > nav_to_target_distance:
            path_final_check.append(0)
        
        else:
            path_final_check.append(1)
    
    
    # find the manipulation safe point
    if 1 in path_final_check:
        manipulation_safe_point = len(path_final_check) - list(reversed(path_final_check)).index(1) - 1
    else: 
        manipulation_safe_point = -1

def calc_current_pose_on_path():

    global path
    
    global manipulation_safe_point
    
    if len(path) != 0: 
        global current_pose_on_path
        current_pose_on_path = [0.0 for i in range(0)]
    
        for i in range(path_step):
            current_pose_on_path.append((np.sqrt(np.power((path[i][0] - current_x), 2) + np.power((path[i][1] - current_y), 2))))
                
        if distance2goal < nav_to_target_distance*2:
            manipulation_safe_point = -1
        
        if manipulation_safe_point == -1:
            manipulation_signal = 1
            pub.publish(manipulation_signal)
        
        elif manipulation_safe_point < np.argmin(current_pose_on_path):
            manipulation_signal = 1
            pub.publish(manipulation_signal)
            
        else:
            manipulation_signal = 0
            pub.publish(manipulation_signal)


def navigation_check(msg):
    
    global orientation_start
    orientation_start = msg.data


if __name__ == '__main__':

    try:
        rospy.init_node('curvature_check')
        target_x = rospy.get_param('target_x')
        target_y = rospy.get_param('target_y')
        target_z = rospy.get_param('target_z')

        path_length = 0
        path_step = 0
        current_x = 0
        current_y = 0
        path = [[0.0, 0.0] for i in range(0)]
        path_final_check = [0.0 for i in range(0)]
        curvature = [0.0 for i in range(0)]
        curvature_check = [0.0 for i in range(0)]
        
        orientation_start = 0
        manipulation_safe_point = 0
        
        calc_nav_to_target_distance()
        sub1 = rospy.Subscriber("/odometry/filtered", Odometry, get_current_pose)
        sub2 = rospy.Subscriber("/move_base/NavfnROS/plan", Path, get_path_data)
        sub3 = rospy.Subscriber("navigation_check", Int8, navigation_check)
        pub = rospy.Publisher("manipulation_signal", Int8, queue_size=1)

        while not rospy.is_shutdown():
            
            if(orientation_start == 1):
                rospy.loginfo("path curvature check finished.")
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Test Finished.")
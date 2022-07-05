#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import math
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8

GROUP_NAME_ARM = rospy.get_param("/setting/Manipulator_Name")        # arm group name in moveit setup assistant
#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"
FIXED_FRAME = rospy.get_param("/setting/Manipulator_Fixed_frame")
group = [moveit_commander.MoveGroupCommander(GROUP_NAME_ARM)] 
pose_goal = Pose()


class Move():

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)        
        rospy.init_node("manipulation",anonymous=True)

        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot_cmd = moveit_commander.RobotCommander()

        self.robot_arm = moveit_commander.MoveGroupCommander(GROUP_NAME_ARM)
        #robot_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        self.robot_arm.set_goal_orientation_tolerance(0.005)
        self.robot_arm.set_planning_time(5)
        self.robot_arm.set_num_planning_attempts(5)

        rospy.sleep(2)
        # Allow replanning to increase the odds of a solution
        self.robot_arm.allow_replanning(True)
        self.robot_arm.set_named_target("default")
        self.robot_arm.go(wait=True)
        rospy.sleep(1)
        
    def move_code(self):
        
        global nav_to_target_distance
        global target_size
        global target_z
        
        if manipulation_signal == 1:
            pose_goal.orientation.x = 0.707
            pose_goal.orientation.y = 0.0
            pose_goal.orientation.z = 0.707
            pose_goal.orientation.w = 0.001
            pose_goal.position.x = nav_to_target_distance - target_size
            pose_goal.position.y = 0.0003
            pose_goal.position.z = target_z - base_link_offset
            group[0].set_pose_target(pose_goal)
            group[0].go(True)
            
            rospy.sleep(1)
            
        elif manipulation_signal == 0: 
            self.robot_arm.set_named_target("default")
            self.robot_arm.go(wait=True)
                   
            rospy.sleep(1)
        
        else:
            self.robot_arm.set_named_target("default")
            self.robot_arm.go(wait=True)
            
            rospy.sleep(1)
            
    def final_pose_adjustment(self):
        
        pose_goal.orientation.x = 0.707
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.707
        pose_goal.orientation.w = 0.001
        pose_goal.position.x = nav_to_target_distance - target_size
        pose_goal.position.y = 0.0003
        pose_goal.position.z = target_z - base_link_offset
        group[0].set_pose_target(pose_goal)
        group[0].go(True)
        
        rospy.sleep(1)
        
    def pose_for_grasping(self):
        
        pose_goal.orientation.x = 0.707
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.707
        pose_goal.orientation.w = 0.001
        pose_goal.position.x = nav_to_target_distance
        pose_goal.position.y = 0.0003
        pose_goal.position.z = target_z - base_link_offset
        group[0].set_pose_target(pose_goal)
        group[0].go(True)
        
        rospy.sleep(1)


def get_target_data():
    
    global target_x
    global target_y
    global target_z
    global target_size
    
    target_x = rospy.get_param('target_x')
    target_y = rospy.get_param('target_y')
    target_z = rospy.get_param('target_z')
    
    target_size = rospy.get_param("/setting/target_size")
    


def get_mobile_manipulator_data():
    
    global target_z
    global base_link_offset
    global nav_to_target_distance
    
    ####################################################################################
    # distance from ground to manipulator's first_joint_height                         #
    a = rospy.get_param("/setting/first_joint_height")                                 #
    # target_z_coordinate                                                              #
    b = target_z                                                                       #
    # distance from mobile_base_lilnk to manipulator_base_link                         #
    c = rospy.get_param("/setting/between_base_link")                                  #
    # manipulator safty zone distance                                                  #
    d = rospy.get_param("/setting/recommended_reach")                                  #
    # height of ground to mobile robot's base_link                                     #
    base_link_offset = rospy.get_param("/setting/base_link_offset")                    #
    ####################################################################################

    if a - b > 0:
        target_theta = math.acos((a-b)/d)
    else:
        target_theta = math.acos((b-a)/d)
    
    nav_to_target_distance = d * math.sin(target_theta) + c
    


def get_manipulation_signal(msg):
    
    global manipulation_signal
    manipulation_signal = msg.data


def navigation_check(msg):
    
    global orientation_start
    orientation_start = msg.data

def orientation_check(msg):
    
    global orientation_end
    orientation_end = msg.data


if __name__=='__main__':
    
    global manipulation_signal
    global orientation_start
    global orientation_end
    manipulation_signal = 0
    orientation_start = 0
    orientation_end = 0
    
    get_target_data()
    get_mobile_manipulator_data()
    
    sub = rospy.Subscriber("manipulation_signal", Int8, get_manipulation_signal)
    sub2 = rospy.Subscriber("navigation_check", Int8, navigation_check)
    sub3 = rospy.Subscriber("orientation_end", Int8, orientation_check)
    
    mo = Move()
    mo.__init__()
    
    
    while not rospy.is_shutdown():
        
        mo.move_code()
        
        if(orientation_start == 1):
            mo.final_pose_adjustment()
        
            if(orientation_end == 1):
                mo.pose_for_grasping()
            
                rospy.loginfo("manipulation finished.")
                break
        
    moveit_commander.roscpp_shutdown()
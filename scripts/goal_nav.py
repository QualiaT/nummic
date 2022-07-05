#!/usr/bin/env python
import rospy
import actionlib
import math

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

def movebase_client():

    global refresh_cycle
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = rospy.get_param("/setting/Nav_header_frame")
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = target_x
    goal.target_pose.pose.position.y = target_y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    rospy.sleep(refresh_cycle)

def distance2D(msg):
    
    px = msg.pose.pose.position.x
    py = msg.pose.pose.position.y

    dx = target_x - px
    dy = target_y - py

    global distance2goal 
    distance2goal = math.sqrt(dx*dx + dy*dy)

def get_current_pose(msg):

    global current_x
    global current_y

    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

def calc_nav_to_target_distance():

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
    ####################################################################################

    if a - b > 0:
        target_theta = math.acos((a-b)/d)
    else:
        target_theta = math.acos((b-a)/d)
    
    nav_to_target_distance = d * math.sin(target_theta) + c


if __name__ == '__main__':

    try:
        rospy.init_node('goal_nav_node')
        target_x = rospy.get_param('target_x')
        target_y = rospy.get_param('target_y')
        target_z = rospy.get_param('target_z')
        refresh_cycle = rospy.get_param("/setting/refresh_cycle")

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        distance2goal = 3
        calc_nav_to_target_distance()

        sub1 = rospy.Subscriber("/odometry/filtered", Odometry, distance2D)

        sub2 = rospy.Subscriber("/odometry/filtered", Odometry, get_current_pose)

        while not rospy.is_shutdown():
            result = movebase_client()
            
            #distance goal setting!!
            if distance2goal <= nav_to_target_distance:
                break
            

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
#!/usr/bin/env python
import rospy
import actionlib
import math
import numpy as np

from std_msgs.msg import Int8
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def movebase_client():

    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = rospy.get_param("/setting/Nav_header_frame")
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = current_x
    goal.target_pose.pose.position.y = current_y

    quaternion = quaternion_from_euler(0, 0, current_theta)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()  

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
    global current_theta

    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

    yaw_x = target_x - current_x
    yaw_y = target_y - current_y
    current_theta = math.atan2(yaw_y, yaw_x)


def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

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
        rospy.init_node('goal_nav_stop_node')
        target_x = rospy.get_param('target_x')
        target_y = rospy.get_param('target_y')
        target_z = rospy.get_param('target_z')

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        distance2goal = 3
        calc_nav_to_target_distance()
        
        print("")
        print("#####nav_to_target_distance is %f#####"%nav_to_target_distance)
        print("")

        sub1 = rospy.Subscriber("/odometry/filtered", Odometry, distance2D)
        sub2 = rospy.Subscriber("/odometry/filtered", Odometry, get_current_pose)
        sub3 = rospy.Subscriber('/odometry/filtered', Odometry, get_rotation)

        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        pub2 = rospy.Publisher('navigation_check',Int8, queue_size=1)

        speed = Twist()

        r = rospy.Rate(4)

        status = 0
        roll = pitch = yaw = 0.0

        r.sleep()
        print("")
        print("Distance to target is %f"%distance2goal)
        print("Safety allowance for manipulating is %f"%(nav_to_target_distance*2))
        print("")



        if distance2goal <= nav_to_target_distance*2:

            print("")
            print("##### case: too close!!! #####")
            print("")

            while not rospy.is_shutdown():
                
                pub2.publish(status)

                #distance goal setting!!
                if distance2goal <= nav_to_target_distance:
                
                    speed.linear.x = 0.0
                    pub.publish(speed)
                    r.sleep()

                    result = movebase_client()
                    status = 1
                    pub2.publish(status)
                    r = rospy.Rate(10)
                    break
                        


        else:

            print("")
            print("##### case: normal!!! #####")
            print("")

            while not rospy.is_shutdown():

                pub2.publish(status)

                #distance goal setting!!
                if distance2goal < nav_to_target_distance*2:
                    
                    # rospy.loginfo("sigmoid based speed control start.")
                    
                    #distance goal setting!! distance2goal - 1 part!
                    desired_speed = 0.2/(np.exp(-12*(distance2goal - nav_to_target_distance)+6)+1) + 0.07
                    speed.linear.x = desired_speed
                    pub.publish(speed)
                    rospy.sleep(0.05)

                    #distance goal setting!!
                    if distance2goal < nav_to_target_distance:
                    
                        speed.linear.x = 0.0
                        pub.publish(speed)
                        r.sleep()

                        result = movebase_client()
                        status = 1
                        pub2.publish(status)
                        r = rospy.Rate(10)
                        break



    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
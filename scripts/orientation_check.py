#!/usr/bin/env python
import rospy
import math

from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from decimal import Decimal



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
    global target_orientation

    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

    yaw_x = target_x - current_x
    yaw_y = target_y - current_y
    target_radians = math.atan2(yaw_y, yaw_x)
    target_degree = math.degrees(target_radians)

    if target_radians < 0:
        target_degree += 360
    
    target_orientation = target_degree


def get_rotation (msg):
    global roll, pitch, yaw
    global yaw_degree
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    degree = math.degrees(yaw)

    if yaw < 0:
        degree += 360
    
    yaw_degree = degree


def navigation_check(msg):
    
    global orientation_start
    orientation_start = msg.data


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
        rospy.init_node('orientation_check')
        target_x = rospy.get_param('target_x')
        target_y = rospy.get_param('target_y')
        target_z = rospy.get_param('target_z')

        orientation_start = 0
        orientation_end = 0
        roll = pitch = yaw = 0.0
        kP = rospy.get_param("/setting/orientation_kP_value")

        distance2goal = 0
        calc_nav_to_target_distance()

        sub1 = rospy.Subscriber("/odometry/filtered", Odometry, distance2D)
        sub2 = rospy.Subscriber("/odometry/filtered", Odometry, get_current_pose)
        sub3 = rospy.Subscriber("/odometry/filtered", Odometry, get_rotation)
        sub4 = rospy.Subscriber("navigation_check", Int8, navigation_check)
        
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        pub2 = rospy.Publisher('orientation_end', Int8, queue_size=1)

        speed = Twist()
        r = rospy.Rate(4)
        r.sleep()

        print_count = 0
        while not rospy.is_shutdown():
            
            if orientation_start == 1:

                r = rospy.Rate(10)
                command = Twist()
                command.linear.x = 0
                
                theta_substract = target_orientation - yaw_degree

                if theta_substract < -180:
                    theta_substrct = (360 + theta_substract)
                
                if theta_substract > 180:
                    theta_substract = (360 - theta_substract) * (-1)

                
                radian_substract = math.radians(theta_substract)
                command.angular.z = kP * (radian_substract)
                pub.publish(command)
                if print_count == 0:
                    print("Target = {}    Current : {}".format(target_orientation,yaw_degree))
                    print("orientation control now......")
                    print("......")
                    print("......")
                    print("......")
                r.sleep()

                if theta_substract < 0:
                    theta_substract = theta_substract * (-1)
                    
                print_count = print_count + 1
                
                if '{:.3f}'.format(Decimal(theta_substract)) < '{:.3f}'.format(Decimal('0.23')):
                    
                    orientation_end = 1
                    pub2.publish(orientation_end)
                    r = rospy.Rate(10)
                    
                    print("")
                    print("############################################################################")
                    print("##################Final Distance to target is %f######################"%distance2goal)
                    print("############################################################################")
                    print("")
                    break


                
    except rospy.ROSInterruptException:
        rospy.loginfo("Orientation check finished.")
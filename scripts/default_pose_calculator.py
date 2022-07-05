#!/usr/bin/env python
import rospy
import numpy as np
import math


def get_value():
  
  global max_z_int    # manipulation z_max value (mm)
  global min_z_int    # manipulation z_min value (mm)
  global r_r_int      # recommended_reach (mm)
  global d_bf_int     # x-axis distance from manipulator's base link to footprint (mm)
  
  max_z = rospy.get_param("/setting/z_max") * 1000
  min_z = rospy.get_param("/setting/z_min") * 1000
  r_r = rospy.get_param("/setting/recommended_reach") * 1000
  d_bf = rospy.get_param("/setting/m_bl_to_ft") * 1000

  max_z_int = int(max_z)
  min_z_int = int(min_z)
  r_r_int = int(r_r)
  d_bf_int = int(d_bf)


def calculate_dp():
  
  dist = np.zeros(shape = (min_z_int, max_z_int), dtype=np.double)
  V = np.zeros(shape = (min_z_int,), dtype=np.double)

  for k in range(0, min_z_int):
    for x in range(0,max_z_int):
      y = math.sqrt(r_r_int*r_r_int - (x - (max_z_int - min_z_int))*(x - (max_z_int - min_z_int)))
      dist[k, x] = math.sqrt(((x - (max_z_int - min_z_int))-k)*((x - (max_z_int - min_z_int))-k) + (y-d_bf_int)*(y-d_bf_int))

  for l in range(0, min_z_int):
    select = dist[l, :]
    var = np.var(select)
    V[l] = math.sqrt(var)


  answer = min(V)
  answer2 = V.tolist().index(answer) + 1
  
  first_joint_height = rospy.get_param("/setting/first_joint_height")
  base_link_offset = rospy.get_param("/setting/base_link_offset")
  dp_z_p = float(answer2) / 1000
  
  answer_final_x = rospy.get_param("/setting/between_base_link") + rospy.get_param("/setting/m_bl_to_ft")
  answer_final_z = first_joint_height + dp_z_p - base_link_offset
  
  print("##############################################################################")
  print("")
  print("")
  print("Your default pose's x-coordinate(based on mobile manipulator's base_link) is: ")
  print(answer_final_x)
  print("")
  print("Your default pose's z-coordinate(based on mobile manipulator's base_link) is: ")
  print(answer_final_z)
  print("")
  print("")
  print("##############################################################################")



if __name__ == '__main__':
  
  rospy.init_node('default_pose_calculator')
  
  get_value()
  calculate_dp()
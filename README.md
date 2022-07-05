# nummic
Navigation path based Universal Mobile Manipulator Integrated Controller (NUMMIC)

## Overview
This is a simultaneous controller for mobile manipulator.
This package is for [ROS](http://wiki.ros.org/ROS/Installation) Melodic.

In order to use this controller, [move_base](http://wiki.ros.org/move_base) & [MoveIt](https://moveit.ros.org/) settings for mobile manipulator must be completed.



### Author:
- **[TaeHyeon Kim](https://github.com/QualiaT), qualiatxr@gmail.com**

**Special Thanks: [Myunghyun Kim](https://github.com/kmh8667), [SungWoo Yang](https://github.com/Sungwwoo), [Sangheum Lee](https://github.com/Shumine) and [GyeongMin Kim](https://github.com/gmkim97)**

**Affiliation: [Human-Robot Interaction LAB](https://khu-hri.weebly.com), Kyung Hee Unviersity, South Korea**



## Default pose setting

- Modify 'nummic/config/controller_param.yaml' according to the spec of the mobile manipulator you want to use

- Run the next launch file
```
$ roslaunch nummic default_pose_cal.launch
```

- Manipulate the end-effector based on the calculated values using MoveIt Commander. In this case, designate the y-coordinate as close to zero as possible.

![01](https://user-images.githubusercontent.com/87522493/177284778-837a73a0-3bdb-47ee-b29d-9f71bf829152.png)


- Click the Joints tab inside the MotionPlanning window and check each joint value.
- Based on the joint values, add a default pose to the srdf file in your mobile manipulator's 'XXX_gripper_moveit_config/config'

![02](https://user-images.githubusercontent.com/87522493/177284817-4e8b5ce0-08e6-437a-a83b-5dcaaa8d3bab.png)





## How to use controller?
```
$ roslaunch nummic controller.launch x:={target's x-coordinate} y:={target's y-coordinate} z:={target's z-coordinate}
```

![readme 01](https://user-images.githubusercontent.com/87522493/177284830-962307b5-243c-493a-a0f8-db9e1fe3e9bd.gif)
  
  
![readme 02](https://user-images.githubusercontent.com/87522493/177284848-cc7ceb9d-b3fa-4624-800f-2bf13dbbe4ee.gif)

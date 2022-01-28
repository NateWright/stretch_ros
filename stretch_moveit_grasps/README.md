![](../images/HelloRobotLogoBar.png)

## Overview

*stretch_moveit_grasps* provides a node that recieves points to move the gripper and recieve a bool array to move the head.

## Setup

Use `rosdep` to install the required packages.

```bash
    cd ~/catkin_ws/src
    git clone https://github.com/NateWright/stretch_ros.git --feature/MoveItGrasps
    git clone https://github.com/pal-robotics/realsense_gazebo_plugin
    cd ~/catkin_ws
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make
```

## Running Demo

##### Start Demo

```bash
    roslaunch stretch_moveit_config demo.launch
    roslaunch stretch_moveit_grasps stretch_node.launch
```

##### Execute commands for robot

Moving Head: Array of bools for direction. Home bool. Step in degrees for head to rotate.

```bash
rostopic pub /move_head stretch_moveit_grasps/stretch_move_bool "{lookLeft: false, lookRight: false, lookUp: false, lookDown: false, home: false, step: 0}"
```

Moving Arm: Takes a geometry_msgs/Pose as arg

```bash
rostopic pub /move_arm geometry_msgs/Pose "position:
  x: 0.0
  y: 0.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0"
```
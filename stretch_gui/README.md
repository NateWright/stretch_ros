![](../images/HelloRobotLogoBar.png)

## Overview

This package is in active development. Proceed with caution.

stretch_gui is meant to utilize all aspects of stretch to navigate and grasps objects. This package utilizes stretch_rtabmap to map. 

# Requirements Gazebo

```shell
cd ~/catkin_ws/src
git clone https://github.com/NateWright/stretch_ros
git clone https://github.com/pal-robotics/realsense_gazebo_plugin
git clone https://github.com/NateWright/stretch_pc
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

# Usage Gazebo

```shell
# Shell 1
roslaunch stretch_rtabmap gazebo.launch
# Shell 2
roslaunch stretch_rtabmap start_rtab.launch sim:=true localization:=false move_base_config:=3d
# Shell 3
roslaunch stretch_pc stretch_pc.launch point_cloud_topic:="/camera/depth/color/points"
# Shell 4
roslaunch stretch_moveit_config move_group.launch
# Shell 5
roslaunch stretch_gui stretch_gui.launch sim:=true
```

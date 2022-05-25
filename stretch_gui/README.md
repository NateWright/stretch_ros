![](../images/HelloRobotLogoBar.png)

## Overview

This package is in active development. Proceed with caution.

stretch_gui is meant to utilize all aspects of stretch to navigate and grasps objects. This package utilizes stretch_rtabmap to map. 

# Requirements

```shell
cd ~/catkin_ws/src
git clone https://github.com/NateWright/stretch_ros.git
git clone https://github.com/pal-robotics/realsense_gazebo_plugin
git clone https://github.com/NateWright/stretch_lineup.git
git clone https://github.com/NateWright/stretch_pc
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

# Usage

```shell
# Shell 1
roslaunch stretch_rtabmap gazebo.launch
# Shell 2
roslaunch stretch_rtabmap start_rtab.launch sim:=true localization:=false move_base_config:=3d
# Shell 3
roslaunch stretch_pc stretch_pc.launch
# Shell 4
roslaunch stretch_lineup stretch_lineup.launch
# Shell 5
rosrun stretch_gui main
```

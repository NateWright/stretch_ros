## Overview

This package is in active development. Proceed with caution.

stretch_gui is meant to utilize all aspects of stretch to navigate and grasps objects. This package utilizes stretch_rtabmap to map.

# Requirements Real Robot

```shell
cd ~/catkin_ws/src

git clone https://github.com/NateWright/stretch_ros -b dev/realBot
git clone https://github.com/NateWright/stretch_pc

cd ~/catkin_ws

rosdep install --from-paths src --ignore-src -r -y

catkin build
```

# Usage

```shell
# Shell 1

roslaunch stretch_rtabmap start_rtab.launch sim:=false localization:=false move_base_config:=3d

# Shell 2

roslaunch stretch_pc stretch_pc.launch.

# Shell 3

roslaunch stretch_gui stretch_interface.launch

# Shell 5

roslaunch stretch_gui stretch_gui.launch
```
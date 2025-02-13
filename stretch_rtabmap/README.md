![](../images/HelloRobotLogoBar.png)

## Overview

This package is in active development. Proceed with caution.

*stretch_rtabmap* provides RTAB-MAPPiNG. This package utilizes rtab-map, move_base, and AMCL to map and drive the stretch RE1 around a space. Running this code will require the robot to be untethered.

## Setup

Use `rosdep` to install the required packages.

```bash
    cd ~/catkin_ws/src
    git clone https://github.com/hello-robot/stretch_ros -b dev/noetic
    git clone https://github.com/pal-robotics/realsense_gazebo_plugin
    cd ~/catkin_ws
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make
```

## Running Demo

### Gazebo

```bash
    roslaunch stretch_rtabmap gazebo.launch
    roslaunch stretch_rtabmap start_rtab.launch sim:=true
    roslaunch stretch_rtabmap rviz_rtab.launch mapping:=true
```

### Stretch RE1
```bash
    roslaunch stretch_rtabmap robot.launch
    roslaunch stretch_rtabmap start_rtab.launch sim:=false
    roslaunch stretch_rtabmap rviz_rtab.launch mapping:=true
```
## Code Status & Development Plans

Move_base_config | Gazebo          | Stretch RE1
-----------------|-----------------|----------------
2d               | Okay            | Good
2d_unkown        | Working         | Working
3d               | Okay            | Good
3d_unkown        | Good            | Good

## License

For license information, please see the LICENSE files.

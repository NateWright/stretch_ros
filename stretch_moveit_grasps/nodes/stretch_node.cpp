#include <ros/ros.h>

#include "stretch_moveit_interface.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "stretch_move_group_interface");
    ros::NodeHandlePtr nh(new ros::NodeHandle());

    ros::AsyncSpinner s(1);
    s.start();

    stretch_moveit_interface str(nh);
    ros::waitForShutdown();
    return 0;
}

#include <ros/ros.h>

#include "StretchInterfaceGazebo.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "stretch_move_group_interface");
    ros::NodeHandlePtr nh(new ros::NodeHandle());

    ros::AsyncSpinner s(1);
    s.start();

    StretchInterfaceGazebo stretch(nh);
    ros::waitForShutdown();
    return 0;
}
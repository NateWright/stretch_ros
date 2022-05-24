#include "movebasestatus.hpp"

MoveBaseStatus::MoveBaseStatus(ros::NodeHandle *nodeHandle) : nh_(nodeHandle) {
    moveBaseStatusSub_ = nh_->subscribe("/move_base/status", 1000, &MoveBaseStatus::moveBaseStatusCallback, this);
    moveBaseStopPub_ = nh_->advertise<actionlib_msgs::GoalID>("/move_base/cancel", 30);
}

MoveBaseStatus::~MoveBaseStatus() {
}

void MoveBaseStatus::run() {
    exec();
}

int MoveBaseStatus::exec() {
    ros::Rate loop_rate(120);
    while (ros::ok() && !isInterruptionRequested()) {
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}

void MoveBaseStatus::moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg) {
    if (!msg.get()->status_list.empty() && msg.get()->status_list.back().status == 1) {
        robotMoving(true);
        return;
    }
    robotMoving(false);
}

void MoveBaseStatus::stopRobot(){
  actionlib_msgs::GoalID stop;
  stop.stamp.sec = 0;
  stop.stamp.nsec = 0;
  stop.id = "";
  moveBaseStopPub_.publish(stop);
}

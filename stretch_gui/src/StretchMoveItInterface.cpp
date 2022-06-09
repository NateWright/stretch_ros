#include "StretchMoveItInterface.hpp"

StretchMoveItInterface::StretchMoveItInterface(ros::NodeHandle* nh) :
     nh_(nh) {
    headTilt_ = nh_->advertise<std_msgs::Float64>("/stretch_interface/head_tilt", 1000);
    headPan_ = nh_->advertise<std_msgs::Float64>("/stretch_interface/head_pan", 1000);
    armLift_ = nh_->advertise<std_msgs::Float64>("/stretch_interface/lift", 1000);
    armExtension_ = nh_->advertise<std_msgs::Float64>("/stretch_interface/lift_extension", 1000);
    gipperYaw_ = nh_->advertise<std_msgs::Float64>("/stretch_interface/wrist_yaw", 1000);
    gripperAperature_ = nh_->advertise<std_msgs::Float64>("/stretch_interface/gripper_opening", 1000);
    moveToThread(this);
}

StretchMoveItInterface::~StretchMoveItInterface() {}

void StretchMoveItInterface::run() {
  QTimer *timer = new QTimer();
  timer->setInterval(15);
  connect(timer, &QTimer::timeout, this, &StretchMoveItInterface::loop);
  timer->start();
    armSetHeight(1);
  exec();
  delete timer;
}

void StretchMoveItInterface::loop() {
  ros::spinOnce();
}

void StretchMoveItInterface::headSetRotation(double degPan, double degTilt) {
  headSetPan(degPan);
  headSetTilt(degTilt);
}

void StretchMoveItInterface::headSetPan(double degPan){
  std_msgs::Float64 msg;
  msg.data = degPan * toRadians;
  headPan_.publish(msg);
}
void StretchMoveItInterface::headSetTilt(double degTilt){
  std_msgs::Float64 msg;
  msg.data = degTilt * toRadians;
  headTilt_.publish(msg);
}
void StretchMoveItInterface::armSetHeight(double metersHeight) {
  std_msgs::Float64 msg;
  msg.data = metersHeight;
  armLift_.publish(msg);
}
void StretchMoveItInterface::armSetReach(double metersReach) {
  std_msgs::Float64 msg;
  msg.data = metersReach;
  armExtension_.publish(msg);
}
void StretchMoveItInterface::gripperSetRotate(double deg) {
  std_msgs::Float64 msg;
  msg.data = deg * toRadians;
  gipperYaw_.publish(msg);
}
void StretchMoveItInterface::gripperSetGrip(double deg) {
  std_msgs::Float64 msg;
  msg.data = deg * toRadians;
  gripperAperature_.publish(msg);
}
void StretchMoveItInterface::homeRobot(){
  headSetTilt();
  headSetPan();
  gripperSetGrip();
  gripperSetRotate();
  armSetHeight();
  armSetReach();
}

void StretchMoveItInterface::headUp() {
  headSetTilt(5);
}
void StretchMoveItInterface::headDown() {
  headSetTilt(-5);
}
void StretchMoveItInterface::headLeft() {
  headSetPan(5);
}
void StretchMoveItInterface::headRight() {
  headSetPan(-5);
}

void StretchMoveItInterface::headHome() {
  headSetRotation();
}

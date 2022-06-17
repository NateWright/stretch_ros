#include "StretchMoveItInterface.hpp"

StretchMoveItInterface::StretchMoveItInterface(ros::NodeHandle* nh) : nh_(nh), tiltAngle_(0), panAngle_(0) {
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
    QTimer* timer = new QTimer();
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

void StretchMoveItInterface::headSetRotation(const double degPan, const double degTilt) {
    headSetPan(degPan);
    headSetTilt(degTilt);
}

void StretchMoveItInterface::headSetPan(const double degPan) {
    panAngle_ = degPan;
    std_msgs::Float64 msg;
    msg.data = degPan * toRadians;
    headPan_.publish(msg);
}
void StretchMoveItInterface::headSetTilt(const double degTilt) {
    tiltAngle_ = degTilt;
    std_msgs::Float64 msg;
    msg.data = degTilt * toRadians;
    headTilt_.publish(msg);
}
void StretchMoveItInterface::armSetHeight(const double metersHeight) {
    std_msgs::Float64 msg;
    msg.data = metersHeight;
    armLift_.publish(msg);
}
void StretchMoveItInterface::armSetReach(const double metersReach) {
    std_msgs::Float64 msg;
    msg.data = metersReach;
    armExtension_.publish(msg);
}
void StretchMoveItInterface::gripperSetRotate(const double deg) {
    std_msgs::Float64 msg;
    msg.data = deg * toRadians;
    gipperYaw_.publish(msg);
}
void StretchMoveItInterface::gripperSetGrip(const double deg) {
    std_msgs::Float64 msg;
    msg.data = deg * toRadians;
    gripperAperature_.publish(msg);
}
void StretchMoveItInterface::homeRobot() {
    headSetTilt();
    headSetPan();
    gripperSetGrip();
    gripperSetRotate();
    armSetHeight();
    armSetReach();
}

void StretchMoveItInterface::headUp() {
    tiltAngle_ += 5;
    headSetTilt(tiltAngle_);
}
void StretchMoveItInterface::headDown() {
    tiltAngle_ -= 5;
    headSetTilt(tiltAngle_);
}
void StretchMoveItInterface::headLeft() {
    panAngle_ += 5;
    headSetPan(panAngle_);
}
void StretchMoveItInterface::headRight() {
    panAngle_ -= 5;
    headSetPan(panAngle_);
}

void StretchMoveItInterface::headHome() {
    headSetRotation();
}
#include "StretchInterfaceGazebo.hpp"

StretchInterfaceGazebo::StretchInterfaceGazebo(ros::NodeHandlePtr nh) : nh_(nh) {
    move_group_interface_arm_ = new moveit::planning_interface::MoveGroupInterface(ARM);
    move_group_interface_arm_->setGoalTolerance(0.01);
    move_group_interface_arm_->setPlanningTime(20.0);
    move_group_interface_arm_->setMaxVelocityScalingFactor(1.0);
    move_group_interface_arm_->setPlanningTime(5);

    move_group_interface_gripper_ = new moveit::planning_interface::MoveGroupInterface(GRIPPER);
    move_group_interface_gripper_->setGoalTolerance(0.01);
    move_group_interface_gripper_->setPlanningTime(20.0);
    move_group_interface_gripper_->setMaxVelocityScalingFactor(1.0);

    move_group_interface_head_ = new moveit::planning_interface::MoveGroupInterface(HEAD);
    move_group_interface_head_->setGoalTolerance(0.01);
    move_group_interface_head_->setPlanningTime(20.0);
    move_group_interface_head_->setMaxVelocityScalingFactor(1.0);

    headTiltSubscriber_ = nh_->subscribe("/stretch_interface/head_tilt", 1000, &StretchInterfaceGazebo::headTiltCallback, this);
    headPanSubscriber_ = nh_->subscribe("/stretch_interface/head_pan", 1000, &StretchInterfaceGazebo::headPanCallback, this);
    armLiftSubscriber_ = nh_->subscribe("/stretch_interface/lift", 1000, &StretchInterfaceGazebo::armLiftCallback, this);
    armExtensionSubscriber_ = nh_->subscribe("/stretch_interface/lift_extension", 1000, &StretchInterfaceGazebo::armExtensionCallback, this);
    gripperYawSubscriber_ = nh_->subscribe("/stretch_interface/wrist_yaw", 1000, &StretchInterfaceGazebo::gripperYawCallback, this);
    gripperApertureSubscriber_ = nh_->subscribe("/stretch_interface/gripper_opening", 1000, &StretchInterfaceGazebo::gripperApertureCallback, this);
}
StretchInterfaceGazebo::~StretchInterfaceGazebo() {
    delete move_group_interface_arm_;
    delete move_group_interface_gripper_;
    delete move_group_interface_head_;
}

void StretchInterfaceGazebo::headTiltCallback(const std_msgs::Float64::ConstPtr msg) {
    const std::lock_guard<std::mutex> lock(robotLock_);
    ros::AsyncSpinner s(1);
    s.start();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group_interface_head_->getCurrentState();
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface_head_->getCurrentState()->getJointModelGroup(HEAD);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions.at(1) = msg->data;

    move_group_interface_head_->setJointValueTarget(joint_group_positions);
    move_group_interface_head_->plan(my_plan);
    move_group_interface_head_->execute(my_plan);
}
void StretchInterfaceGazebo::headPanCallback(const std_msgs::Float64::ConstPtr msg) {
    const std::lock_guard<std::mutex> lock(robotLock_);
    ros::AsyncSpinner s(1);
    s.start();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group_interface_head_->getCurrentState();
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface_head_->getCurrentState()->getJointModelGroup(HEAD);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions.at(0) = msg->data;

    move_group_interface_head_->setJointValueTarget(joint_group_positions);
    move_group_interface_head_->plan(my_plan);
    move_group_interface_head_->execute(my_plan);
}
void StretchInterfaceGazebo::armLiftCallback(const std_msgs::Float64::ConstPtr msg) {
    const std::lock_guard<std::mutex> lock(robotLock_);
    ros::AsyncSpinner s(1);
    s.start();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group_interface_arm_->getCurrentState();
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface_arm_->getCurrentState()->getJointModelGroup(ARM);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions.at(0) = msg->data;

    move_group_interface_arm_->setJointValueTarget(joint_group_positions);
    move_group_interface_arm_->plan(my_plan);
    move_group_interface_arm_->execute(my_plan);
}
void StretchInterfaceGazebo::armExtensionCallback(const std_msgs::Float64::ConstPtr msg) {
    const std::lock_guard<std::mutex> lock(robotLock_);
    ros::AsyncSpinner s(1);
    s.start();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group_interface_arm_->getCurrentState();
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface_arm_->getCurrentState()->getJointModelGroup(ARM);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    double dist = msg->data / 4;
    for (int i = 1; i < 5; i++) {
        joint_group_positions.at(i) = dist;
    }

    move_group_interface_arm_->setJointValueTarget(joint_group_positions);
    move_group_interface_arm_->plan(my_plan);
    move_group_interface_arm_->execute(my_plan);
}
void StretchInterfaceGazebo::gripperYawCallback(const std_msgs::Float64::ConstPtr msg) {
    const std::lock_guard<std::mutex> lock(robotLock_);
    ros::AsyncSpinner s(1);
    s.start();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // Get current state
    moveit::core::RobotStatePtr arm_state = move_group_interface_arm_->getCurrentState();
    // Get joint values for the group
    const moveit::core::JointModelGroup* joint_model_group_arm = move_group_interface_arm_->getCurrentState()->getJointModelGroup(ARM);
    std::vector<double> arm_positions;
    arm_state->copyJointGroupPositions(joint_model_group_arm, arm_positions);

    arm_positions.back() = msg->data;

    move_group_interface_arm_->setJointValueTarget(arm_positions);
    // Planning
    move_group_interface_arm_->plan(my_plan);
    // Executing
    move_group_interface_arm_->execute(my_plan);
}
void StretchInterfaceGazebo::gripperApertureCallback(const std_msgs::Float64::ConstPtr msg) {
    const std::lock_guard<std::mutex> lock(robotLock_);
    ros::AsyncSpinner s(1);
    s.start();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // Get current state
    moveit::core::RobotStatePtr gripper_state = move_group_interface_gripper_->getCurrentState();
    // Get joint values for the group
    const moveit::core::JointModelGroup* joint_model_group_gripper = move_group_interface_gripper_->getCurrentState()->getJointModelGroup(GRIPPER);
    std::vector<double> gripper_positions;
    gripper_state->copyJointGroupPositions(joint_model_group_gripper, gripper_positions);

    gripper_positions.front() = msg->data;
    gripper_positions.back() = msg->data;

    move_group_interface_gripper_->setJointValueTarget(gripper_positions);
    // Planning
    move_group_interface_gripper_->plan(my_plan);
    // Executing
    move_group_interface_gripper_->execute(my_plan);
}
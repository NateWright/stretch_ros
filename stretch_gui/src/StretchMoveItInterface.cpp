#include "StretchMoveItInterface.hpp"

StretchMoveItInterface::StretchMoveItInterface(ros::NodeHandle* nh) :
     nh_(nh) {
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

    armSub_ = nh_->subscribe("/stretch_moveit_grasps/arm", 1000, &StretchMoveItInterface::stretchArmCallback, this);
    headSub_ = nh_->subscribe("/stretch_moveit_grasps/head", 1000, &StretchMoveItInterface::stretchHeadCallback, this);
    cameraAdjustment_ = nh_->advertise<stretch_moveit_grasps::stretch_move_bool>("/stretch_moveit_grasps/head", 30);
    moveToThread(this);
}

StretchMoveItInterface::~StretchMoveItInterface() {
    delete move_group_interface_arm_;
    delete move_group_interface_gripper_;
    delete move_group_interface_head_;
}

void StretchMoveItInterface::run() {
  QTimer *timer = new QTimer();
  connect(timer, &QTimer::timeout, this, &StretchMoveItInterface::loop);
  timer->start();
  exec();
  delete timer;
}

void StretchMoveItInterface::loop() {
  ros::spinOnce();
}
void StretchMoveItInterface::stretchArmCallback(const geometry_msgs::Pose::ConstPtr& target_pose1) {
    // Start spinner to be able to access Current position
    ros::AsyncSpinner s(1);
    // Get current pose
    s.start();
    geometry_msgs::PoseStamped ps = move_group_interface_arm_->getCurrentPose();
    geometry_msgs::Pose target_pose2 = ps.pose;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // target_pose2.position.z = target_pose2.position.z + target_pose1.position.z;
    // target_pose2.position.y = target_pose2.position.y - target_pose1.position.y;
    target_pose2.position.z = target_pose1.get()->position.z;
    target_pose2.position.y = target_pose1.get()->position.y;

    // Setting pose
    move_group_interface_arm_->setPoseTarget(target_pose2);
    // Planning
    bool success = (move_group_interface_arm_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // Executing
    move_group_interface_arm_->execute(my_plan);
    s.stop();
}

void StretchMoveItInterface::stretchHeadCallback(const stretch_moveit_grasps::stretch_move_bool msg) {
    // Start spinner to be able to access Current position
    ros::AsyncSpinner s(1);
    s.start();
    //
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // Get current state
    moveit::core::RobotStatePtr current_state = move_group_interface_head_->getCurrentState();
    // Get joint values for the group
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface_head_->getCurrentState()->getJointModelGroup(HEAD);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    double step = (msg.step == 0) ? DEG5 : msg.step * 0.01745;

    // Pan Head joint_group_positions.at(0) positive left | negative right
    // Tilt head joint_group_positions.at(1)  positive up | negative down
    if (joint_group_positions.size() == 2) {
        if (msg.home) {
            joint_group_positions.at(0) = 0;
            joint_group_positions.at(1) = 0;
        } else {
            if (msg.lookLeft) {
                joint_group_positions.at(0) += step;
            } else if (msg.lookRight) {
                joint_group_positions.at(0) -= step;
            }

            if (msg.lookUp) {
                joint_group_positions.at(1) += step;
            } else if (msg.lookDown) {
                joint_group_positions.at(1) -= step;
            }
        }
    }

    // Setting pose
    move_group_interface_head_->setJointValueTarget(joint_group_positions);
    // Planning
    bool success = (move_group_interface_head_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // Executing
    move_group_interface_head_->execute(my_plan);
    s.stop();
}

void StretchMoveItInterface::headSetRotation(double degPan, double degTilt) {
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // Get current state
    moveit::core::RobotStatePtr current_state = move_group_interface_head_->getCurrentState();
    // Get joint values for the group
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface_head_->getCurrentState()->getJointModelGroup(HEAD);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions.at(0) = degPan * M_PI / 180;
    joint_group_positions.at(1) = degTilt * M_PI / 180;

    // Setting pose
    move_group_interface_head_->setJointValueTarget(joint_group_positions);
    // Planning
    move_group_interface_head_->plan(my_plan);
    // Executing
    move_group_interface_head_->execute(my_plan);
}

void StretchMoveItInterface::headSetPan(double degPan){
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // Get current state
  moveit::core::RobotStatePtr current_state = move_group_interface_head_->getCurrentState();
  // Get joint values for the group
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface_head_->getCurrentState()->getJointModelGroup(HEAD);
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions.at(0) = degPan * M_PI / 180;

  // Setting pose
  move_group_interface_head_->setJointValueTarget(joint_group_positions);
  // Planning
  move_group_interface_head_->plan(my_plan);
  // Executing
  move_group_interface_head_->execute(my_plan);
}
void StretchMoveItInterface::headSetTilt(double degTilt){
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // Get current state
  moveit::core::RobotStatePtr current_state = move_group_interface_head_->getCurrentState();
  // Get joint values for the group
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface_head_->getCurrentState()->getJointModelGroup(HEAD);
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions.at(1) = degTilt * M_PI / 180;

  // Setting pose
  move_group_interface_head_->setJointValueTarget(joint_group_positions);
  // Planning
  move_group_interface_head_->plan(my_plan);
  // Executing
  move_group_interface_head_->execute(my_plan);
}
void StretchMoveItInterface::armSetHeight(double height) {
    /*
     * Moving arm to propper height
     */
    // Get current pose
    geometry_msgs::PoseStamped ps = move_group_interface_arm_->getCurrentPose();
    geometry_msgs::Pose target_pose = ps.pose;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    target_pose.position.z = height;

    // Setting pose
    move_group_interface_arm_->setPoseTarget(target_pose);
    // Planning
    move_group_interface_arm_->plan(my_plan);
    // Executing
    move_group_interface_arm_->execute(my_plan);
}
void StretchMoveItInterface::armSetReach(double distance) {
    /*
     * Moving arm to propper height
     */
    // Get current pose
    geometry_msgs::PoseStamped ps = move_group_interface_arm_->getCurrentPose();
    geometry_msgs::Pose target_pose = ps.pose;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    target_pose.position.y = -distance;

    // Setting pose
    move_group_interface_arm_->setPoseTarget(target_pose);
    // Planning
    move_group_interface_arm_->plan(my_plan);
    // Executing
    move_group_interface_arm_->execute(my_plan);
}
void StretchMoveItInterface::gripperSetRotate(double deg) {
    /*
     * Orientating Gripper
     */
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // Get current state
    moveit::core::RobotStatePtr arm_state = move_group_interface_arm_->getCurrentState();
    // Get joint values for the group
    const moveit::core::JointModelGroup* joint_model_group_arm = move_group_interface_arm_->getCurrentState()->getJointModelGroup(ARM);
    std::vector<double> arm_positions;
    arm_state->copyJointGroupPositions(joint_model_group_arm, arm_positions);

    arm_positions.back() = deg * M_PI / 180;

    move_group_interface_arm_->setJointValueTarget(arm_positions);
    // Planning
    move_group_interface_arm_->plan(my_plan);
    // Executing
    move_group_interface_arm_->execute(my_plan);
}
void StretchMoveItInterface::gripperSetGrip(double deg) {
    /*
     * Opening gripper jaws
     */
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // Get current state
    moveit::core::RobotStatePtr gripper_state = move_group_interface_gripper_->getCurrentState();
    // Get joint values for the group
    const moveit::core::JointModelGroup* joint_model_group_gripper = move_group_interface_gripper_->getCurrentState()->getJointModelGroup(GRIPPER);
    std::vector<double> gripper_positions;
    gripper_state->copyJointGroupPositions(joint_model_group_gripper, gripper_positions);

    gripper_positions.front() = deg * M_PI / 180;
    gripper_positions.back() = deg * M_PI / 180;

    move_group_interface_gripper_->setJointValueTarget(gripper_positions);
    // Planning
    move_group_interface_gripper_->plan(my_plan);
    // Executing
    move_group_interface_gripper_->execute(my_plan);
}

void StretchMoveItInterface::move(direction d) {
    stretch_moveit_grasps::stretch_move_bool robot;
    switch (d) {
        case Up:
            robot.lookUp = true;
            break;
        case Down:
            robot.lookDown = true;
            break;
        case Left:
            robot.lookLeft = true;
            break;
        case Right:
            robot.lookRight = true;
        case Home:
            robot.home = true;
    }
    robot.step = 5;
    cameraAdjustment_.publish(robot);
}

void StretchMoveItInterface::headUp() { move(Up); }
void StretchMoveItInterface::headDown() { move(Down); }
void StretchMoveItInterface::headLeft() { move(Left); }
void StretchMoveItInterface::headRight() { move(Right); }

void StretchMoveItInterface::headHome() { move(Home); }

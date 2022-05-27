#include "stretch_moveit_interface.hpp"

stretch_moveit_interface::stretch_moveit_interface(ros::NodeHandlePtr nh) : nh_(nh) {
    move_group_interface_arm = new moveit::planning_interface::MoveGroupInterface(ARM);
    move_group_interface_arm->setGoalTolerance(0.01);
    move_group_interface_arm->setPlanningTime(20.0);

    move_group_interface_head = new moveit::planning_interface::MoveGroupInterface(HEAD);
    move_group_interface_head->setGoalTolerance(0.01);
    move_group_interface_head->setPlanningTime(20.0);
    move_group_interface_head->setMaxVelocityScalingFactor(1.0);

    armSub = nh_.get()->subscribe("/stretch_moveit_grasps/arm", 1000, &stretch_moveit_interface::stretchArmCallback, this);
    headSub = nh_.get()->subscribe("/stretch_moveit_grasps/head", 1000, &stretch_moveit_interface::stretchHeadCallback, this);
}

stretch_moveit_interface::~stretch_moveit_interface() {
}

void stretch_moveit_interface::stretchArmCallback(const geometry_msgs::Pose::ConstPtr& target_pose1) {
    const std::lock_guard<std::mutex> lock(execution_);
    // Start spinner to be able to access Current position
    ros::AsyncSpinner s(1);
    // Get current pose
    s.start();
    geometry_msgs::PoseStamped ps = move_group_interface_arm->getCurrentPose();
    geometry_msgs::Pose target_pose2 = ps.pose;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // target_pose2.position.z = target_pose2.position.z + target_pose1.position.z;
    // target_pose2.position.y = target_pose2.position.y - target_pose1.position.y;
    target_pose2.position.z = target_pose1.get()->position.z;
    target_pose2.position.y = target_pose1.get()->position.y;

    // Setting pose
    move_group_interface_arm->setPoseTarget(target_pose2);
    // Planning
    bool success = (move_group_interface_arm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // Executing
    move_group_interface_arm->execute(my_plan);
    s.stop();
}

void stretch_moveit_interface::stretchHeadCallback(const stretch_moveit_grasps::stretch_move_bool msg) {
    const std::lock_guard<std::mutex> lock(execution_);
    // Start spinner to be able to access Current position
    ros::AsyncSpinner s(1);
    s.start();
    //
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // Get current state
    moveit::core::RobotStatePtr current_state = move_group_interface_head->getCurrentState();
    // Get joint values for the group
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface_head->getCurrentState()->getJointModelGroup(HEAD);
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
    move_group_interface_head->setJointValueTarget(joint_group_positions);
    // Planning
    bool success = (move_group_interface_head->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // Executing
    move_group_interface_head->execute(my_plan);
    s.stop();
}
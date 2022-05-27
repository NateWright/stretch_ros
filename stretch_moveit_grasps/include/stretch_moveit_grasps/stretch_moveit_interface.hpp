#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <stretch_moveit_grasps/stretch_move_bool.h>

#include <mutex>

#pragma once

const std::string HEAD = "stretch_head", ARM = "stretch_arm";
const double DEG5 = 0.0872665;

class stretch_moveit_interface {
   private:
    ros::NodeHandlePtr nh_;
    ros::Subscriber armSub;
    ros::Subscriber headSub;

    std::mutex execution_;
    moveit::planning_interface::MoveGroupInterface *move_group_interface_arm;
    moveit::planning_interface::MoveGroupInterface *move_group_interface_head;

    void stretchArmCallback(const geometry_msgs::Pose::ConstPtr &target_pose1);
    void stretchHeadCallback(const stretch_moveit_grasps::stretch_move_bool msg);

   public:
    explicit stretch_moveit_interface(ros::NodeHandlePtr nh);
    ~stretch_moveit_interface();
};
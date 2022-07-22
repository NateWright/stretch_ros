#ifndef StretchInterfaceGazebo_HPP
#define StretchInterfaceGazebo_HPP

#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <mutex>

const std::string HEAD = "stretch_head",
                  ARM = "stretch_arm",
                  GRIPPER = "stretch_gripper";

class StretchInterfaceGazebo {
   public:
    explicit StretchInterfaceGazebo(ros::NodeHandlePtr nh);
    ~StretchInterfaceGazebo();

    void move();

   private:
    ros::NodeHandlePtr nh_;
    ros::NodeHandlePtr subHandle_;
    ros::CallbackQueue queue_;
    ros::Subscriber headTiltSubscriber_;
    ros::Subscriber headPanSubscriber_;
    ros::Subscriber armLiftSubscriber_;
    ros::Subscriber armExtensionSubscriber_;
    ros::Subscriber gripperYawSubscriber_;
    ros::Subscriber gripperApertureSubscriber_;

    moveit::planning_interface::MoveGroupInterface *move_group_interface_arm_;
    moveit::planning_interface::MoveGroupInterface *move_group_interface_head_;
    moveit::planning_interface::MoveGroupInterface *move_group_interface_gripper_;

    float headPan_,
        headTilt_,
        armLift_,
        armExtension_,
        gripperYaw_,
        gripperAperture_;
    bool moveHead_,
        moveArm_,
        moveGripper_;

    std::mutex robotLock_;

    void headTiltCallback(const std_msgs::Float64::ConstPtr msg);
    void headPanCallback(const std_msgs::Float64::ConstPtr msg);
    void armLiftCallback(const std_msgs::Float64::ConstPtr msg);
    void armExtensionCallback(const std_msgs::Float64::ConstPtr msg);
    void gripperYawCallback(const std_msgs::Float64::ConstPtr msg);
    void gripperApertureCallback(const std_msgs::Float64::ConstPtr msg);
};

#endif  // StretchInterfaceGazebo_HPP

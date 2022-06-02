#ifndef STRETCHMOVEITINTERFACE_HPP
#define STRETCHMOVEITINTERFACE_HPP

#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <stretch_moveit_grasps/stretch_move_bool.h>

#include <QObject>
#include <QThread>

enum direction { Up, Down, Left, Right, Home };

const std::string HEAD = "stretch_head", ARM = "stretch_arm", GRIPPER = "stretch_gripper";
const double DEG5 = 0.0872665;

class StretchMoveItInterface : public QThread {
    Q_OBJECT
   public:
    explicit StretchMoveItInterface(ros::NodeHandle *nh);
    ~StretchMoveItInterface();
    void run() override;

   protected:
    int exec();

   private:
    ros::NodeHandle *nh_;
    ros::Subscriber armSub_;
    ros::Subscriber headSub_;
    ros::Publisher cameraAdjustment_;

    moveit::planning_interface::MoveGroupInterface *move_group_interface_arm_;
    moveit::planning_interface::MoveGroupInterface *move_group_interface_head_;
    moveit::planning_interface::MoveGroupInterface *move_group_interface_gripper_;

    void move(direction d);

    void stretchArmCallback(const geometry_msgs::Pose::ConstPtr &target_pose1);
    void stretchHeadCallback(const stretch_moveit_grasps::stretch_move_bool msg);

   public slots:
    void headSetRotation(double degPan = 0, double degTilt = 0);
    void headSetPan(double degPan = 0);
    void headSetTilt(double degTilt = 0);
    void armSetHeight(double meters = 0.2);
    void armSetReach(double meters = 0);
    void gripperSetRotate(double deg = 180);
    void gripperSetGrip(double deg = 0);
    void headUp();
    void headDown();
    void headLeft();
    void headRight();
    void headHome();
};

#endif  // STRETCHMOVEITINTERFACE_HPP

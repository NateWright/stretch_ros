#ifndef GRASPNODE_HPP
#define GRASPNODE_HPP

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <QDebug>
#include <QThread>
#include <QWidget>
#include <atomic>

enum Stage{HOLD, HOME, GRASP, NEXT};

class GraspNode : public QThread {
    Q_OBJECT
   public:
    explicit GraspNode(ros::NodeHandle *nh);
    ~GraspNode();
    void run() override;

   protected:
    int exec();

   private:
    ros::NodeHandle *nh_;
    ros::Publisher resetPub_;
    ros::Publisher cmdVelPub_;
    ros::Publisher cmdArmPub_;
    ros::Subscriber centerPointSub_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener *tfListener_;

    geometry_msgs::PointStamped::Ptr point_;

    std::atomic<Stage> stage_;

    void centerPointCallback(const geometry_msgs::PointStamped::ConstPtr &input);
    void lineUp();
    void homeRobot();

   signals:
    //    void navigateToPoint(const geometry_msgs::PointStamped::ConstPtr& input);
    void headSetRotation(double degPan = 0, double degTilt = 0);
    void headSetPan(double degPan = 0);
    void headSetTilt(double degTilt = 0);
    void armSetHeight(double meters = 0.2);
    void armSetReach(double meters = 0);
    void gripperSetRotate(double deg = 180);
    void gripperSetGrip(double deg = 0);
   public slots:
    void doLineUp();
    void doHomeRobot();
};

#endif  // GRASPNODE_HPP

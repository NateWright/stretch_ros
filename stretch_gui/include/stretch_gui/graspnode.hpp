#ifndef GRASPNODE_HPP
#define GRASPNODE_HPP

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <QDebug>
#include <QObject>
#include <QPainter>
#include <QPoint>
#include <QThread>
#include <QWidget>

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
    tf2_ros::TransformListener* tfListener_;

    geometry_msgs::PointStamped::ConstPtr point_;

    bool showPoint_;
    QPoint item_;

    QPixmap camera_;
    QPixmap cameraOutputRotated_;

    void centerPointCallback(const geometry_msgs::PointStamped::ConstPtr& input);

   signals:
    void imgUpdate(const QPixmap &);
    void displayWaitMessage(bool);
    void navigateToPoint(const geometry_msgs::PointStamped::ConstPtr& input);
    void checkPointInRange(const geometry_msgs::PointStamped::ConstPtr& input);
    void validPoint();
    void invalidPoint();
   public slots:
    void enablePoint();
    void disablePoint();
    void setImage(const QPixmap &);
    void setPoint(const QPoint);
    void checkPointReturn(bool b);
    void lineUp();
};

#endif  // GRASPNODE_HPP

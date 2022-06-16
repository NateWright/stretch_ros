#ifndef MAPSUBSCRIBER_H
#define MAPSUBSCRIBER_H

#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <QDebug>
#include <QTimer>
#include <QGraphicsScene>
#include <QImage>
#include <QObject>
#include <QPainter>
#include <QPen>
#include <QPixmap>
#include <QPoint>
#include <QSize>
#include <QThread>
#include <cmath>

using std::vector;

class MapSubscriber : public QThread {
    Q_OBJECT
   public:
    explicit MapSubscriber(ros::NodeHandle *nodeHandle);
    ~MapSubscriber();
    void run() override;

   private:
    ros::NodeHandle *nh_;
    ros::Subscriber mapSub_;
    ros::Subscriber posSub_;
    ros::Publisher movePub_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener *tfListener_;

    QImage map_;
    QImage mapCopy_;
    QPixmap outputMap_;

    bool drawPos_;
    QPoint origin_;
    QPoint robotPos_;
    double robotRot_;

    double resolution_;

    bool drawMouseArrow_;
    QPoint mousePressLocation_;
    QPoint mousePressCurrentLocation_;

    geometry_msgs::PoseStamped robotHome_;


    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void posCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void loop();
 signals:
   void mapUpdate(const QPixmap &);
   void validPoint();
   void invalidPoint();
   void homeSet(bool);

 public slots:
   void moveRobot(QPoint press, QPoint release, QSize screen);
   void moveRobotLoc(const geometry_msgs::PoseStamped::Ptr pose);
   void mousePressInitiated(QPoint press, QSize screen);
   void mousePressCurrentLocation(QPoint loc, QSize screen);
   void navigateToPoint(const geometry_msgs::PointStamped::ConstPtr& input);
   void checkPointInRange(const geometry_msgs::PointStamped::ConstPtr& input);
   void setHome();
   void setHomeIfNone();
   void navigateHome();
   void enableMapping();
   void disableMapping();
   void rotate(int degrees);
   void rotateLeft(int degrees = 5);
   void rotateRight(int degrees = 5);
   void drive(double metere);
};

QPoint translateScreenToMap(QPoint p, QSize screen, QSize map);

#endif  // MAPSUBSCRIBER_H

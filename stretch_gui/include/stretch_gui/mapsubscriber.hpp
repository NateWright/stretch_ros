#ifndef MAPSUBSCRIBER_H
#define MAPSUBSCRIBER_H

#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <QDebug>
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
    MapSubscriber(ros::NodeHandle *nodeHandle);
    ~MapSubscriber();
    void run() override;

   signals:
    void mapUpdate(const QPixmap &);
    void validPoint();
    void invalidPoint();

   public slots:
    void moveRobot(QPoint press, QPoint release, QSize screen);
    void mousePressInitiated(QPoint press, QSize screen);
    void mousePressCurrentLocation(QPoint loc, QSize screen);
    void navigateToPoint(const geometry_msgs::PointStamped::ConstPtr& input);
    void checkPointInRange(const geometry_msgs::PointStamped::ConstPtr& input);

   protected:
    int exec();

   private:
    ros::NodeHandle *nh_;
    ros::Subscriber mapSub_;
    ros::Subscriber posSub_;
    ros::Publisher movePub_;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;

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


    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void posCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

QPoint translateScreenToMap(QPoint p, QSize screen, QSize map);

#endif  // MAPSUBSCRIBER_H

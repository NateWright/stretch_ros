#ifndef MAPSUBSCRIBER_H
#define MAPSUBSCRIBER_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <QByteArray>
#include <QFile>
#include <QGraphicsScene>
#include <QImage>
#include <QObject>
#include <QPainter>
#include <QPair>
#include <QPixmap>
#include <QThread>

using std::vector;


class mapsubscriber : public QThread {
    Q_OBJECT
   public:
    mapsubscriber(ros::NodeHandle *nodeHandle);
    ~mapsubscriber();
    void run() override;

   signals:
    void mapUpdate(const QPixmap &);

   public slots:
    void moveRobot(int x, int y, int width, int height);

   private:
    ros::NodeHandle *nh_;
    ros::Subscriber mapSub_;
    ros::Subscriber posSub_;
    ros::Publisher movePub_;

    QImage map_;
    QPixmap outputMap_;

    QPoint origin_;
    QPoint pos_;

    double resolution_;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;

    void mapCallback(const nav_msgs::OccupancyGrid msg);
    void posCallback(const nav_msgs::Odometry msg);
};

#endif  // MAPSUBSCRIBER_H

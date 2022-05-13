#ifndef MAPSUBSCRIBER_H
#define MAPSUBSCRIBER_H

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <QByteArray>
#include <QFile>
#include <QGraphicsScene>
#include <QImage>
#include <QObject>
#include <QPixmap>
#include <QThread>
#include <vector>

using std::vector;

class mapsubscriber : public QThread {
    Q_OBJECT
   public:
    mapsubscriber(ros::NodeHandle *nodeHandle);
    ~mapsubscriber();
    void run() override;

   signals:
    void mapUpdate(const QPixmap &);

   private:
    QGraphicsScene scene_;
    ros::NodeHandle *nh_;
    ros::Subscriber mapSub_;
    QPixmap img_;
    int *data;
    QByteArray qb;

    void callback(const nav_msgs::OccupancyGrid msg);
};

#endif  // MAPSUBSCRIBER_H

#ifndef GRASPNODE_HPP
#define GRASPNODE_HPP

#include <QThread>
#include <QObject>
#include <QWidget>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

class GraspNode : public QThread
{
  Q_OBJECT
  public:
    explicit GraspNode(ros::NodeHandle *nh);
    ~GraspNode();
    void run() override;
  private:
    ros::NodeHandle *nh_;
    ros::Publisher resetPub_;
    ros::Subscriber itemSub_;

    QImage camera_;
    QPixmap cameraOutput_;
    QPixmap cameraOutputRotated_;

    void itemCloudCallback(const sensor_msgs::PointCloud2 input);
  signals:
    void imgUpdate(const QPixmap &);
  public slots:
    void reset();



};

#endif // GRASPNODE_HPP

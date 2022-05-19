#ifndef GRASPNODE_HPP
#define GRASPNODE_HPP

#include <QThread>
#include <QObject>
#include <QWidget>
#include <QPoint>
#include <QPainter>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_ros/point_cloud.h>

class GraspNode : public QThread
{
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
    ros::Subscriber centerPointSub_;

    QPoint item_;

    QPixmap camera_;
    QPixmap cameraOutputRotated_;

    void centerPointCallback(const geometry_msgs::PointStamped input);
  signals:
    void imgUpdate(const QPixmap &);
  public slots:
    void setImage(const QPixmap &);
    void reset();
    void setPoint(QPoint);



};

#endif // GRASPNODE_HPP

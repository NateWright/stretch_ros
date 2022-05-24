#ifndef GRASPNODE_HPP
#define GRASPNODE_HPP

#include <geometry_msgs/PointStamped.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

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
    ros::Subscriber centerPointSub_;

    bool showPoint_;
    QPoint item_;

    QPixmap camera_;
    QPixmap cameraOutputRotated_;

    void centerPointCallback(const geometry_msgs::PointStamped::ConstPtr& input);
   signals:
    void imgUpdate(const QPixmap &);
    void pointReceived(bool);
   public slots:
    void setImage(const QPixmap &);
    void reset();
    void setPoint(QPoint);
};

#endif  // GRASPNODE_HPP

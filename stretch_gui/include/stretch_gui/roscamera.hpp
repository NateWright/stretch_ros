#ifndef ROSCAMERA_HPP
#define ROSCAMERA_HPP

#include <geometry_msgs/PointStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/distances.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stretch_moveit_grasps/stretch_move_bool.h>
#include <geometry_msgs/PointStamped.h>

#include <QColor>
#include <QImage>
#include <QObject>
#include <QPixmap>
#include <QPainter>
#include <QThread>
#include <QWidget>
#include <QPoint>
#include <QSize>
#include <QDebug>
#include <cmath>
#include <QTimer>

class RosCamera : public QThread {
    Q_OBJECT
   public:
    explicit RosCamera(ros::NodeHandle *nh);
    ~RosCamera();
    void run() override;

   private:
    ros::NodeHandle *nh_;
    ros::Subscriber colorCameraSub_;
    ros::Subscriber segmentedCameraSub_;
    ros::Subscriber centerPointSub_;
    ros::Publisher pointPick_;

    std::string frameId_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    QImage camera_;
    QPixmap cameraOutput_;
    QPixmap cameraOutputRotated_;
    QPixmap cameraOutputRotatedWithPoint_;

    QPoint centerPoint_;
    bool showCenterPoint_;

    void cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& pc);
    void centerPointCallback(const geometry_msgs::PointStamped::ConstPtr& point);
    void loop();

   signals:
    void imgUpdate(const QPixmap &);
    void imgUpdateWithPoint(const QPixmap &);
    void checkPointInRange(const geometry_msgs::PointStamped::ConstPtr &output);
    void clickSuccess();
    void clickFailure();
    void clickInitiated();
   public slots:
    void sceneClicked(QPoint press, QPoint release, QSize screen);
    void showCenterPoint();
    void hideCenterPoint();
};

#endif  // ROSCAMERA_HPP

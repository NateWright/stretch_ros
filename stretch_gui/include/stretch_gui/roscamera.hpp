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
#include <QThread>
#include <QWidget>
#include <QPoint>
#include <QSize>
#include <QDebug>
#include <cmath>

enum direction { Up, Down, Left, Right, Home };

class RosCamera : public QThread {
    Q_OBJECT
   public:
    explicit RosCamera(ros::NodeHandle *nh);
    ~RosCamera();
    void run() override;

   protected:
    int exec();

   private:
    ros::NodeHandle *nh_;
    ros::Subscriber colorCameraSub_;
    ros::Subscriber segmentedCameraSub_;
    ros::Subscriber centerPointSub_;
    ros::Publisher cameraAdjustment_;
    ros::Publisher pointPick_;

    std::string frameId_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    QImage camera_;
    QPixmap cameraOutput_;
    QPixmap cameraOutputRotated_;

    void cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& pc);
    void centerPointCallback(const geometry_msgs::PointStamped::ConstPtr& point);
    void move(direction d);

   signals:
    void imgUpdate(const QPixmap &);
    void objectCenterPixel(const QPoint);
    void clickSuccess();
    void clickFailure(bool);
   public slots:
    void moveUp();
    void moveDown();
    void moveLeft();
    void moveRight();
    void moveHome();
    void sceneClicked(QPoint press, QPoint release, QSize screen);
};

#endif  // ROSCAMERA_HPP

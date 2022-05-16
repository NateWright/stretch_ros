#ifndef ROSCAMERA_HPP
#define ROSCAMERA_HPP

#include <geometry_msgs/PointStamped.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stretch_moveit_grasps/stretch_move_bool.h>

#include <QColor>
#include <QImage>
#include <QObject>
#include <QPixmap>
#include <QThread>
#include <QWidget>

enum direction { Up, Down, Left, Right, Home };

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
    ros::Publisher cameraAdjustment_;
    ros::Publisher pointPick_;

    std::string frameId_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    QImage camera_;
    QPixmap cameraOutput_;
    QPixmap cameraOutputRotated_;

    void cameraCallback(const sensor_msgs::PointCloud2 pc);
    void move(direction d);

   signals:
    void imgUpdate(const QPixmap &);
   public slots:
    void moveUp();
    void moveDown();
    void moveLeft();
    void moveRight();
    void moveHome();
    void sceneClicked(int x, int y, int width, int height);
};

#endif  // ROSCAMERA_HPP

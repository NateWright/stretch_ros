#ifndef ROSCAMERA_HPP
#define ROSCAMERA_HPP

#include <QThread>
#include <QObject>
#include <QWidget>
#include <QPixmap>
#include <QImage>
#include <QColor>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <stretch_moveit_grasps/stretch_move_bool.h>

enum direction{Up, Down, Left, Right, Home};

class rosCamera : public QThread
{
  Q_OBJECT
  public:
    explicit rosCamera(ros::NodeHandle *nh);
    ~rosCamera();
    void run() override;

  private:
    ros::NodeHandle *nh_;
    ros::Subscriber colorCameraSub_;
    ros::Subscriber segmentedCameraSub_;
    ros::Publisher cameraAdjustment_;

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

#endif // ROSCAMERA_HPP

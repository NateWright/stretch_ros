#ifndef MOVEBASESTATUS_HPP
#define MOVEBASESTATUS_HPP

#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <ros/ros.h>

#include <QDebug>
#include <QObject>
#include <QThread>
#include <QTimer>
#include <QWidget>

class MoveBaseStatus : public QThread {
    Q_OBJECT
   public:
    explicit MoveBaseStatus(ros::NodeHandlePtr nodeHandle);
    ~MoveBaseStatus();
    void run() override;

   signals:
    void robotMoving(bool);

   public slots:
    void stopRobot();

   private:
    ros::NodeHandlePtr nh_;
    ros::Subscriber moveBaseStatusSub_;
    ros::Publisher moveBaseStopPub_;

    void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
    void loop();
};

#endif  // MOVEBASESTATUS_HPP
#ifndef MOVEBASESTATUS_HPP
#define MOVEBASESTATUS_HPP

#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalID.h>
#include <ros/ros.h>

#include <QObject>
#include <QThread>
#include <QWidget>
#include <QDebug>

class MoveBaseStatus : public QThread {
    Q_OBJECT
   public:
    MoveBaseStatus(ros::NodeHandle *nodeHandle);
    ~MoveBaseStatus();
    void run() override;

   signals:
    void robotMoving(bool);

   public slots:
    void stopRobot();

   protected:
    int exec();

   private:
    ros::NodeHandle *nh_;
    ros::Subscriber moveBaseStatusSub_;
    ros::Publisher moveBaseStopPub_;

    void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
};

#endif  // MOVEBASESTATUS_HPP

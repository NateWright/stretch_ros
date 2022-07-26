#ifndef STRETCHMOVEITINTERFACE_HPP
#define STRETCHMOVEITINTERFACE_HPP

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <QDebug>
#include <QObject>
#include <QThread>
#include <QTimer>

const double toRadians = M_PI / 180;

class StretchMoveItInterface : public QThread {
    Q_OBJECT
   public:
    explicit StretchMoveItInterface(ros::NodeHandlePtr nh);
    ~StretchMoveItInterface();
    void run() override;

    std::pair<int, int> getHeadPanTilt();

   private:
    ros::NodeHandlePtr nh_;
    ros::Publisher headTilt_;
    ros::Publisher headPan_;
    ros::Publisher armLift_;
    ros::Publisher armExtension_;
    ros::Publisher gipperYaw_;
    ros::Publisher gripperAperture_;

    ros::AsyncSpinner *spinner_;

    int panAngle_;
    int tiltAngle_;

   public slots:
    void headSetRotation(const double degPan = 0, const double degTilt = 0);
    void headSetPan(const double degPan = 0);
    void headSetTilt(const double degTilt = 0);
    void armSetHeight(const double metersHeight = 0.2);
    void armSetReach(const double metersReach = 0);
    void gripperSetRotate(const double deg = 180);
    void gripperSetGrip(const double deg = 0);
    void homeRobot();
    void headUp();
    void headDown();
    void headLeft();
    void headRight();
    void headHome();
};

#endif  // STRETCHMOVEITINTERFACE_HPP
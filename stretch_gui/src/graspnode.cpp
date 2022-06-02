#include "graspnode.hpp"

GraspNode::GraspNode(ros::NodeHandle* nh) : nh_(nh), stage_(HOLD) {
    resetPub_ = nh_->advertise<std_msgs::Bool>("/stretch_pc/reset", 30);
    cmdVelPub_ = nh_->advertise<geometry_msgs::Twist>("/stretch_diff_drive_controller/cmd_vel", 1000);
    cmdArmPub_ = nh_->advertise<geometry_msgs::Pose>("/stretch_moveit_grasps/arm", 1000);
    centerPointSub_ = nh_->subscribe("/stretch_pc/centerPoint", 30, &GraspNode::centerPointCallback, this);

    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
    point_.reset(new geometry_msgs::PointStamped());
    moveToThread(this);
}

GraspNode::~GraspNode() {
    delete tfListener_;
}

void GraspNode::run() {
  QTimer *timer = new QTimer();
  connect(timer, &QTimer::timeout, this, &GraspNode::loop);
  timer->start();
  exec();
  delete timer;
}
void GraspNode::loop(){
  ros::spinOnce();
}

void GraspNode::centerPointCallback(const geometry_msgs::PointStamped::ConstPtr& input) {
    geometry_msgs::PointStamped point = tfBuffer_.transform(*input.get(), "map");
    point_.reset(new geometry_msgs::PointStamped());
    *point_.get() = point;
}

void GraspNode::doLineUp(){
  lineUp();
  stage_ = GRASP;
}
void GraspNode::lineUp() {
    std::string targetFrame = "map",
                sourceFrame = "base_link";

    geometry_msgs::TransformStamped transBaseToMap = tfBuffer_.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
      double x = point_.get()->point.x - transBaseToMap.transform.translation.x,
             y = point_.get()->point.y - transBaseToMap.transform.translation.y;

    ros::AsyncSpinner s(1);

    s.start();

    emit headSetPan(-90);
    emit armSetHeight(point_.get()->point.z);
    emit gripperSetRotate(0);
    emit gripperSetGrip(30);
    emit armSetReach(sqrt(x*x + y*y));

    s.stop();

    //  const double offset = -M_PI/2;
    //  geometry_msgs::PointStamped point = tfBuffer_.transform(*point_.get(), targetFrame);
    //  double angle = atan2(y, x);

    //  double yaw = tf2::getYaw(transBaseLinkToMap.transform.rotation) + offset;

    //  geometry_msgs::Twist rotate;
    //  rotate.angular.z = 0.5;
    //  ros::Rate loop_rate(10);

    //  while (ros::ok() && fabs(angle - yaw) > 0.10) {
    //      transBaseLinkToMap = tfBuffer_.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
    //      yaw = tf2::getYaw(transBaseLinkToMap.transform.rotation) + offset;

    //      if (yaw - angle < 0) {
    //          rotate.angular.z = 0.5;
    //      } else {
    //          rotate.angular.z = -0.5;
    //      }
    //      cmdVelPub_.publish(rotate);
    //      loop_rate.sleep();
    //  }
}

void GraspNode::doHomeRobot(){
  homeRobot();
  stage_ = HOME;
}
void GraspNode::homeRobot() {
    ros::AsyncSpinner s(1);

    s.start();

    emit headSetPan();
    emit gripperSetGrip();
    emit gripperSetRotate();
    emit armSetHeight();
    emit armSetReach();

    s.stop();
}

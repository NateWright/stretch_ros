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
  timer->setInterval(15);
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
void GraspNode::lineUp() {
    emit disableMapping();
    std::string targetFrame = "map",
                sourceFrame = "base_link";

    geometry_msgs::TransformStamped transBaseToMap = tfBuffer_.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
    homePose_.reset(new geometry_msgs::PoseStamped());
    homePose_.get()->header.frame_id = "map";
    homePose_.get()->pose.position.x = transBaseToMap.transform.translation.x;
    homePose_.get()->pose.position.y = transBaseToMap.transform.translation.y;
    homePose_.get()->pose.position.z = transBaseToMap.transform.translation.z;
    homePose_.get()->pose.orientation = transBaseToMap.transform.rotation;

    double x = point_.get()->point.x - transBaseToMap.transform.translation.x,
           y = point_.get()->point.y - transBaseToMap.transform.translation.y;

    geometry_msgs::PoseStamped::Ptr pose(new geometry_msgs::PoseStamped());

    pose.get()->header.frame_id = "map";
    pose.get()->pose.position.x = transBaseToMap.transform.translation.x;
    pose.get()->pose.position.y = transBaseToMap.transform.translation.y;
    pose.get()->pose.position.z = transBaseToMap.transform.translation.z;

    tf2::Quaternion q;
    q.setRPY(0,0, atan(y/x) + M_PI/2);
    pose.get()->pose.orientation = tf2::toMsg(q);

    emit navigate(pose);

    emit headSetPan(-90);
    emit armSetHeight(point_.get()->point.z);
    emit gripperSetRotate(0);
    emit gripperSetGrip(30);
//    emit armSetReach(sqrt(x*x + y*y));
    emit graspDone(true);
}

void GraspNode::returnObject(){
  emit headSetTilt();
  emit headSetPan();
  emit gripperSetRotate(90);
  emit armSetReach();
  emit armSetHeight();
  emit enableMapping();
  emit navigateHome();
}

void GraspNode::home() {
  emit headSetPan();
  emit gripperSetGrip();
  emit gripperSetRotate();
  emit armSetReach();
  emit armSetHeight();
  emit enableMapping();
  emit navigate(homePose_);
}

#include "graspnode.hpp"

GraspNode::GraspNode(ros::NodeHandle* nh) : nh_(nh), stage_(HOLD) {
    resetPub_ = nh_->advertise<std_msgs::Bool>("/stretch_pc/reset", 30);
    // cmdVelPub_ = nh_->advertise<geometry_msgs::Twist>("/stretch_diff_drive_controller/cmd_vel", 1000);
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
    ros::AsyncSpinner s(1);
    s.start();
    emit disableMapping();
    std::string targetFrame = "map",
                sourceFrame = "base_link";

    geometry_msgs::TransformStamped transBaseToMap = tfBuffer_.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
    homePose_.reset(new geometry_msgs::PoseStamped());
    homePose_->header.frame_id = "map";
    homePose_->pose.position.x = transBaseToMap.transform.translation.x;
    homePose_->pose.position.y = transBaseToMap.transform.translation.y;
    homePose_->pose.position.z = transBaseToMap.transform.translation.z;
    homePose_->pose.orientation = transBaseToMap.transform.rotation;

    double x = point_->point.x - transBaseToMap.transform.translation.x,
           y = point_->point.y - transBaseToMap.transform.translation.y;
    qDebug() << x*x + y*y;

    geometry_msgs::PoseStamped::Ptr pose(new geometry_msgs::PoseStamped());

    pose->header.frame_id = "map";
    pose->pose.position.x = transBaseToMap.transform.translation.x;
    pose->pose.position.y = transBaseToMap.transform.translation.y;
    pose->pose.position.z = transBaseToMap.transform.translation.z;

    tf2::Quaternion q;
    q.setRPY(0,0, atan(y/x) - (85 * M_PI/180));
    pose->pose.orientation = tf2::toMsg(q);

    emit navigate(pose);

    ros::Duration d(0.5);

    d.sleep();
    emit headSetPan(-90);
    qDebug() << "head set";
    d.sleep();
    emit armSetHeight(point_->point.z + 0.05);
    qDebug() << "arm height set";
    d.sleep();
    emit gripperSetRotate(0);
    qDebug() << "gripper rotate set";
    d.sleep();
    emit gripperSetGrip(30);
    qDebug() << "gripper width set";
    d.sleep();
    emit armSetHeight(point_->point.z);
    d.sleep();
    emit armSetReach(sqrt(x*x + y*y) - 0.35);
    emit graspDone(true);
}

void GraspNode::releaseObject(){
  emit gripperSetGrip(30);
}

void GraspNode::returnObject(){
  ros::Duration d(1.0);
  emit gripperSetGrip();
  d.sleep();
  emit armSetHeight(point_->point.z + 0.05);
  d.sleep();
  emit gripperSetRotate(90);
  d.sleep();
  emit armSetReach();
  d.sleep();
  emit headSetTilt();
  d.sleep();
  emit headSetPan();
  d.sleep();
  emit armSetHeight();
  d.sleep();
  emit enableMapping();
  d.sleep();
  emit navigateHome();
}

void GraspNode::home() {
  ros::Duration d(0.25);
  emit armSetReach();
  d.sleep();
  emit headSetPan();
  d.sleep();
  emit gripperSetGrip();
  d.sleep();
  emit gripperSetRotate();
  d.sleep();
  emit armSetHeight();
  d.sleep();
  emit enableMapping();
  d.sleep();
  d.sleep();
  emit navigate(homePose_);
}

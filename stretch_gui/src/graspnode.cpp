#include "graspnode.hpp"

GraspNode::GraspNode(ros::NodeHandle *nh) : nh_(nh), showPoint_(false){
    resetPub_ = nh_->advertise<std_msgs::Bool>("/stretch_pc/reset", 30);
    cmdVelPub_ = nh_->advertise<geometry_msgs::Twist>("/stretch_diff_drive_controller/cmd_vel", 1000);
    cmdArmPub_ = nh_->advertise<geometry_msgs::Pose>("/stretch_moveit_grasps/arm", 1000);
    centerPointSub_ = nh_->subscribe("/stretch_pc/centerPoint", 30, &GraspNode::centerPointCallback, this);
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
    item_ = QPoint(0,0);
    point_.reset(new geometry_msgs::PointStamped());
}

GraspNode::~GraspNode() {
  delete tfListener_;
}

void GraspNode::run() {
  exec();
}

int GraspNode::exec(){
  ros::Rate loop_rate(20);
  while (ros::ok() && !isInterruptionRequested()) {
      ros::spinOnce();

      if(camera_.width() > 0){
        QImage img = camera_.toImage();
        if(showPoint_){
          QPainter painter(&img);
          painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
          painter.setBrush(QBrush(QColor(Qt::red), Qt::SolidPattern));
          painter.setPen(Qt::NoPen);
          painter.drawEllipse(item_, 20, 20);
          painter.end();
        }
        camera_ = QPixmap::fromImage(img);

        emit imgUpdate(camera_);
      }

      loop_rate.sleep();
  }
  return 0;
}

void GraspNode::centerPointCallback(const geometry_msgs::PointStamped::ConstPtr& input) {
  point_ = input;
}
void GraspNode::enablePoint(){
  showPoint_ = true;
}
void GraspNode::disablePoint(){
  showPoint_ = false;
}
void GraspNode::setImage(const QPixmap &input){
  camera_ = input.copy();
}

void GraspNode::setPoint(const QPoint pixel){
  item_ = pixel;
  emit checkPointInRange(point_);
}

void GraspNode::checkPointReturn(bool b){
  if(b){
    showPoint_ = true;
    emit validPoint();
  } else{
    emit invalidPoint();
  }

  emit displayWaitMessage(false);
}

void GraspNode::lineUp(){
  geometry_msgs::Pose pose;
  pose.position.z = 1.0;
  cmdArmPub_.publish(pose);

//  std::string targetFrame = "map",
//              sourceFrame = "base_link";
//  const double offset = -M_PI/2;
//  geometry_msgs::PointStamped point = tfBuffer_.transform(*point_.get(), targetFrame);
//  geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer_.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
//  double x = point.point.x - transBaseLinkToMap.transform.translation.x,
//         y = point.point.y - transBaseLinkToMap.transform.translation.y;
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

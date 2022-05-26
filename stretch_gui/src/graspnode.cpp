#include "graspnode.hpp"

GraspNode::GraspNode(ros::NodeHandle *nh) : nh_(nh), showPoint_(false){
    resetPub_ = nh_->advertise<std_msgs::Bool>("/stretch_pc/reset", 30);
    lineUpPub_ = nh_->advertise<geometry_msgs::PointStamped>("/stretch_gui/lineUpPoint", 30);
    centerPointSub_ = nh_->subscribe("/stretch_pc/centerPoint", 30, &GraspNode::centerPointCallback, this);
    item_ = QPoint(0,0);
    point_.reset(new geometry_msgs::PointStamped());
}

GraspNode::~GraspNode() {}

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

void GraspNode::reset() {
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
  showPoint_ = false;
  lineUpPub_.publish(point_);
}

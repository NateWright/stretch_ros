#include "graspnode.hpp"

GraspNode::GraspNode(ros::NodeHandle *nh) : nh_(nh) {
    resetPub_ = nh_->advertise<std_msgs::Bool>("/stretch_pc/reset", 1);
    centerPointSub_ = nh_->subscribe("/stretch_pc/centerPoint", 30, &GraspNode::centerPointCallback, this);
    item_ = QPoint(0,0);
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
        QPainter painter(&img);
        painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
        painter.setBrush(QBrush(QColor(Qt::red), Qt::SolidPattern));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(item_, 20, 20);
        painter.end();

        camera_ = QPixmap::fromImage(img);

        emit imgUpdate(camera_);
      }

      loop_rate.sleep();
  }
  return 0;
}

void GraspNode::centerPointCallback(const geometry_msgs::PointStamped input) {

}

void GraspNode::reset() {
    std_msgs::Bool b;
    b.data = 1;
    resetPub_.publish(b);
}

void GraspNode::setImage(const QPixmap &input){
  camera_ = input.copy();
}

void GraspNode::setPoint(QPoint input){
  item_ = input;
}

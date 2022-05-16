#include "roscamera.hpp"

rosCamera::rosCamera(ros::NodeHandle *nh): nh_(nh){
  colorCameraSub_ = nh_->subscribe("/camera/depth/color/points", 30, &rosCamera::cameraCallback, this);
  cameraAdjustment_ = nh_->advertise<stretch_moveit_grasps::stretch_move_bool>("/move_head", 30);

}
rosCamera::~rosCamera(){

}

void rosCamera::run() {
    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        emit imgUpdate(cameraOutputRotated_);

        loop_rate.sleep();
    }
}

void rosCamera::cameraCallback(const sensor_msgs::PointCloud2 pc){
  camera_ = QImage(pc.width, pc.height, QImage::Format_RGB888);

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(pc, pcl_pc2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

  pcl::PointXYZRGB point;
  for(int y = 0; y < pc.height; y++){
    for(int x = 0; x < pc.width; x++){
        point = cloud.get()->at(x,y);
      camera_.setPixel(x,y, QColor(point.r, point.g, point.b).rgb());
    }
  }
  cameraOutput_ = QPixmap::fromImage(camera_);
  cameraOutputRotated_= cameraOutput_.transformed(QTransform().rotate(90));

}

void rosCamera::move(direction d){
  stretch_moveit_grasps::stretch_move_bool robot;
  switch(d){
    case Up:
      robot.lookUp = true;
    break;
  case Down:
    robot.lookDown = true;
    break;
  case Left:
    robot.lookLeft = true;
    break;
  case Right:
    robot.lookRight = true;
  case Home:
    robot.home = true;
  }
  robot.step = 5;
  cameraAdjustment_.publish(robot);

}

void rosCamera::moveUp(){
  move(Up);
}
void rosCamera::moveDown(){
  move(Down);
}
void rosCamera::moveLeft(){
  move(Left);
}
void rosCamera::moveRight(){
  move(Right);
}

void rosCamera::moveHome(){
  move(Home);
}

void rosCamera::sceneClicked(int x, int y, int width, int height){

}

#include "roscamera.hpp"

RosCamera::RosCamera(ros::NodeHandle *nh) : nh_(nh) {
    colorCameraSub_ = nh_->subscribe("/camera/depth/color/points", 30, &RosCamera::cameraCallback, this);
//    colorCameraSub_ = nh_->subscribe("/stretch_pc/pointcloud", 30, &RosCamera::cameraCallback, this);
    cameraAdjustment_ = nh_->advertise<stretch_moveit_grasps::stretch_move_bool>("/stretch_moveit_grasps/head", 30);
    pointPick_ = nh->advertise<geometry_msgs::PointStamped>("/clicked_point", 30);
    centerPointSub_ = nh_->subscribe("/stretch_pc/centerPoint", 30, &RosCamera::centerPointCallback, this);
}
RosCamera::~RosCamera() {}

void RosCamera::run() {
  exec();
}

int RosCamera::exec(){
  ros::Rate loop_rate(20);
  while (ros::ok() && !isInterruptionRequested()) {
      ros::spinOnce();
      emit imgUpdate(cameraOutputRotated_);

      loop_rate.sleep();
  }
  return 0;
}

void RosCamera::cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& pc) {
    frameId_ = pc.get()->header.frame_id;

    const int width = pc.get()->width,
              height = pc.get()->height;
    camera_ = QImage(width, height, QImage::Format_RGB888);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pc.get(), pcl_pc2);

    cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_);

    pcl::PointXYZRGB point;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            point = cloud_.get()->at(x, y);
            camera_.setPixel(x, y, QColor(point.r, point.g, point.b).rgb());
        }
    }
    cameraOutput_ = QPixmap::fromImage(camera_);
    cameraOutputRotated_ = cameraOutput_.transformed(QTransform().rotate(90));
}

void RosCamera::centerPointCallback(const geometry_msgs::PointStamped::ConstPtr& point){
  pcl::PointXYZ p1, curr;
  p1 = pcl::PointXYZ(point.get()->point.x, point.get()->point.y, 0);
  float smallest = pcl::euclideanDistance(p1, pcl::PointXYZ(cloud_.get()->at(0,0).x, cloud_.get()->at(0,0).y, 0));
  float currDistance;

  const int width = cloud_.get()->width,
            height = cloud_.get()->height;
  QPoint min(height,0);

  for(int y = 0; y < height; y++){
    for(int x = 0; x < width; x++){
      curr = pcl::PointXYZ(cloud_.get()->at(x,y).x, cloud_.get()->at(x,y).y, 0);
      currDistance = pcl::euclideanDistance(p1, curr);
      if(std::isnan(smallest)){
        min = QPoint(height - y,x);
        smallest = currDistance;
        continue;
      }
      if(currDistance < smallest){
        min = QPoint(height - y,x);
        smallest = currDistance;
      }
    }
  }
  emit objectCenterPixel(min);
}

void RosCamera::move(direction d) {
    stretch_moveit_grasps::stretch_move_bool robot;
    switch (d) {
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

void RosCamera::moveUp() { move(Up); }
void RosCamera::moveDown() { move(Down); }
void RosCamera::moveLeft() { move(Left); }
void RosCamera::moveRight() { move(Right); }

void RosCamera::moveHome() { move(Home); }

void RosCamera::lookAtArm() {
  qDebug() << "looking at arm";
  stretch_moveit_grasps::stretch_move_bool robot;
  robot.lookRight = true;
  robot.step = 90;
  cameraAdjustment_.publish(robot);
}

void RosCamera::sceneClicked(QPoint press, QPoint release, QSize screen) {
  int locX = press.x();
  int locY = press.y();

  geometry_msgs::PointStamped point;
  point.header.frame_id = frameId_;

  try{
    pcl::PointXYZRGB p = cloud_.get()->at(locY, cloud_.get()->height - locX);

    if(std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)){
      qDebug() << "click fail";
      emit clickFailure();
      throw(std::runtime_error("Point contains NaN"));
    }

    point.point.x = p.x;
    point.point.y = p.y;
    point.point.z = p.z;

    pointPick_.publish(point);
    emit clickSuccess();
  }catch(...){

  }
}

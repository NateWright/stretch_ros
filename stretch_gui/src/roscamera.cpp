#include "roscamera.hpp"

RosCamera::RosCamera(ros::NodeHandle *nh) : nh_(nh), showCenterPoint_(false) {
    colorCameraSub_ = nh_->subscribe("/camera/depth/color/points", 30, &RosCamera::cameraCallback, this);
    segmentedCameraSub_ = nh_->subscribe("/stretch_pc/pointcloud", 30, &RosCamera::segmentedCameraCallback, this);
    pointPick_ = nh->advertise<geometry_msgs::PointStamped>("/clicked_point", 30);
    centerPointSub_ = nh_->subscribe("/stretch_pc/centerPoint", 30, &RosCamera::centerPointCallback, this);
    moveToThread(this);
}
RosCamera::~RosCamera() {}

void RosCamera::run() {
  QTimer *timer = new QTimer();
  timer->setInterval(15);
  connect(timer, &QTimer::timeout, this, &RosCamera::loop);
  timer->start();
  exec();
  delete timer;
}

void RosCamera::loop(){
  ros::spinOnce();

//  QImage img = cameraOutputRotated_.toImage();
//  if (showCenterPoint_) {
//      QPainter painter(&img);
//      painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
//      painter.setBrush(QBrush(QColor(Qt::red), Qt::SolidPattern));
//      painter.setPen(Qt::NoPen);
//      painter.drawEllipse(centerPoint_, 20, 20);
//      painter.end();
//    emit imgUpdateWithPoint(cameraOutputRotatedWithPoint_);
//  }
//  cameraOutputRotatedWithPoint_ = QPixmap::fromImage(img);
//  emit imgUpdateWithPoint(cameraOutputRotatedWithPoint_);
  emit imgUpdate(cameraOutputRotated_);
}

void RosCamera::cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pc) {
    cloud_ = pc;

    const int width = pc->width,
              height = pc->height;
    camera_ = QImage(height, width, QImage::Format_RGB888);
//    camera_ = QImage(width, height, QImage::Format_RGB888);

    pcl::PointXYZRGB point;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            point = pc.get()->at(x, y);
            camera_.setPixel(height - 1 - y, x, QColor(point.r, point.g, point.b).rgb());
//            camera_.setPixel(x, y, QColor(point.r, point.g, point.b).rgb());
        }
    }
    cameraOutputRotated_ = QPixmap::fromImage(camera_);
}
void RosCamera::segmentedCameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pc){
  const int width = cloud_->width,
            height = cloud_->height;
  QImage img = cameraOutputRotated_.toImage();

  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud (cloud_);
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);

  QRgb red = QColor(Qt::red).rgb();
  for(pcl::PointXYZRGB p: *pc){
    kdtree.nearestKSearch(p, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    int pos = pointIdxNKNSearch[0];
    img.setPixel(height - 1 - pos/width, pos % width, red);
  }
  cameraOutputRotatedWithPoint_ = QPixmap::fromImage(img);
  emit imgUpdateWithPoint(cameraOutputRotatedWithPoint_);
}

void RosCamera::centerPointCallback(const geometry_msgs::PointStamped::ConstPtr& point){
  const int width = cloud_->width,
            height = cloud_->height;
  if (width == 0 || height == 0){
    emit checkPointInRange(point);
  }

  pcl::PointXYZRGB p;
  p.x = point->point.x;
  p.y = point->point.y;
  p.z = point->point.z;

  centerPoint_ = findClosest(p);

  emit checkPointInRange(point);
  return;
}

//void RosCamera::centerPointCallback(const geometry_msgs::PointStamped::ConstPtr& point){
//  pcl::PointXYZ p1, curr;
//  p1 = pcl::PointXYZ(point.get()->point.x, point.get()->point.y, 0);
//  float smallest = NAN;
//  float currDistance = 0;

//  const int width = cloud_.get()->width,
//            height = cloud_.get()->height;
//  QPoint min(height,0);

//  for(int y = 0; y < height; y++){
//    for(int x = 0; x < width; x++){
//      curr = pcl::PointXYZ(cloud_.get()->at(x,y).x, cloud_.get()->at(x,y).y, 0);
//      currDistance = pcl::euclideanDistance(p1, curr);
//      if(std::isnan(smallest)){
//        min = QPoint(height - y,x);
//        smallest = currDistance;
//        continue;
//      }
//      if(currDistance < smallest){
////        min = QPoint(height - y,x);
//        min = QPoint(x , y);
//        smallest = currDistance;
//      }
//    }
//  }
//  centerPoint_ = min;
//  emit checkPointInRange(point);
//}


void RosCamera::sceneClicked(QPoint press, QPoint release, QSize screen) {
  int locX = press.x();
  int locY = press.y();

  geometry_msgs::PointStamped point;
  point.header.frame_id = cloud_->header.frame_id;

  try{
    if(locY > cloud_.get()->width || locX > cloud_.get()->height){
      throw(std::runtime_error("Not in range"));
    }
    emit clickInitiated();
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
void RosCamera::showCenterPoint(){
  showCenterPoint_ = true;
}
void RosCamera::hideCenterPoint(){
  showCenterPoint_ = false;
}

QPoint RosCamera::findClosest(const pcl::PointXYZRGB p){
  const int width = cloud_->width,
            height = cloud_->height;

  int y = 0, x = 0;
  float bottom, bottomRight, right;
  double smallest = pcl::euclideanDistance(p, cloud_->at(0,0));
  while(y < height - 1){
    while(x < width - 1){
      bottom = pcl::euclideanDistance(p, cloud_->at(x, y + 1));
      bottomRight = pcl::euclideanDistance(p, cloud_->at(x + 1, y + 1));
      right = pcl::euclideanDistance(p, cloud_->at(x + 1, y));
      if(smallest <= bottom && smallest <= bottomRight && smallest <= right){
        return QPoint(height - 1 - y, x);
      }else if(bottomRight <= bottom && bottomRight <= right){
        smallest = bottomRight;
        x = x+1;
        y = y+1;
        centerPoint_ = QPoint(height - 1 - y, x);
      }else if(bottom <= right){
        smallest = bottom;
        y = y+1;
        centerPoint_ = QPoint(height - 1 - y, x);
      }else{
        smallest = right;
        x = x+1;
        centerPoint_ = QPoint(height - 1 - y, x);
      }
    }
  }
  return QPoint(0,0);
}

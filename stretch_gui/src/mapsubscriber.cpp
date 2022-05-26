#include "mapsubscriber.hpp"

MapSubscriber::MapSubscriber(ros::NodeHandle* nodeHandle)
    : nh_(nodeHandle), robotPos_(QPoint(0, 0)), drawPos_(false), drawMouseArrow_(false) {
    mapSub_ = nh_->subscribe("/map", 30, &MapSubscriber::mapCallback, this);
    posSub_ = nh_->subscribe("/stretch_diff_drive_controller/odom", 30, &MapSubscriber::posCallback, this);
    movePub_ = nh_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 30);
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    map_ = QImage(10, 10, QImage::Format_RGB888);
}

void MapSubscriber::run() {
    exec();
}

int MapSubscriber::exec() {
    ros::Rate loop_rate(20);
    while (ros::ok() && !isInterruptionRequested()) {
        ros::spinOnce();

        mapCopy_ = map_.copy();
        QPainter painter(&mapCopy_);
        painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
        if (drawPos_) {
            painter.setPen(Qt::SolidLine);
            painter.setPen(Qt::red);
            painter.setBrush(QBrush(QColor(Qt::red), Qt::SolidPattern));

            int size = 5;
            QPoint p1(robotPos_.x() + size * std::sin(robotRot_ - M_PI / 2), robotPos_.y() + size * std::cos(robotRot_ - M_PI / 2));
            QPoint p2(robotPos_.x() + size * std::sin(robotRot_ - M_PI), robotPos_.y() + size * std::cos(robotRot_ - M_PI));
            QPoint p3(robotPos_.x() + size * std::sin(robotRot_), robotPos_.y() + size * std::cos(robotRot_));

            QPolygon triangle;
            triangle.clear();
            triangle << p1 << p2 << p3;
            painter.drawPolygon(triangle);
        }
        if (drawMouseArrow_) {
            painter.setPen(Qt::SolidLine);
            painter.setPen(Qt::magenta);
            painter.setBrush(QBrush(QColor(Qt::magenta), Qt::SolidPattern));
            painter.drawLine(mousePressLocation_, mousePressCurrentLocation_);
            painter.drawEllipse(mousePressCurrentLocation_, 1, 1);
        }
        // Paint origin
        //      painter.setPen(Qt::NoPen);
        //      painter.setBrush(QBrush(QColor(Qt::green), Qt::SolidPattern));
        //      painter.drawEllipse(origin_, 5, 5);
        painter.end();

        outputMap_ = QPixmap::fromImage(mapCopy_);

        emit mapUpdate(outputMap_);

        loop_rate.sleep();
    }
    return 0;
}

void MapSubscriber::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    const int width = msg.get()->info.width,
              height = msg.get()->info.height;

    map_ = QImage(width, height, QImage::Format_RGB888);
    resolution_ = msg.get()->info.resolution;
    origin_ = QPoint(width + msg.get()->info.origin.position.x / resolution_, -msg.get()->info.origin.position.y / resolution_);

    int val = 0;
    int pos = 0;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            pos = x + width * y;
            if (msg.get()->data[pos] == -1) {
                val = 100;
            } else if (msg.get()->data[pos] == 100) {
                val = 0;
            } else {
                val = 255;
            }
            map_.setPixel(width - 1 - x, y, QColor(val, val, val).rgb());
        }
    }
}

void MapSubscriber::posCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::string source = "map";
    std::string destination = "base_link";

    try {
        geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer.lookupTransform(source, destination, ros::Time(0));

        robotRot_ = tf2::getYaw(transBaseLinkToMap.transform.rotation);

        robotPos_.setX(origin_.x() - transBaseLinkToMap.transform.translation.x / resolution_);
        robotPos_.setY(origin_.y() + transBaseLinkToMap.transform.translation.y / resolution_);

        drawPos_ = true;
    } catch (...) {
    }
}

void MapSubscriber::moveRobot(QPoint press, QPoint release, QSize screen) {
    drawMouseArrow_ = false;
    geometry_msgs::PoseStamped pose;

    QPoint mapLoc = translateScreenToMap(press, screen, map_.size());

    double locX = (origin_.x() - mapLoc.x()) * resolution_,
           locY = (mapLoc.y() - origin_.y()) * resolution_;

    pose.header.frame_id = "map";
    pose.pose.position.x = locX;
    pose.pose.position.y = locY;

    double difX = release.x() - press.x();
    double difY = release.y() - press.y();

    tf2::Vector3 v1(-1, 0, 0);
    v1.normalize();
    tf2::Vector3 v2(difX, difY, 0);
    v2.normalize();

    tf2::Quaternion q = tf2::shortestArcQuat(v1, v2);
    q.setZ(-q.z());

    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    movePub_.publish(pose);
}

void MapSubscriber::mousePressInitiated(QPoint press, QSize screen) {
    mousePressLocation_ = translateScreenToMap(press, screen, map_.size());
}

void MapSubscriber::mousePressCurrentLocation(QPoint loc, QSize screen) {
    mousePressCurrentLocation_ = translateScreenToMap(loc, screen, map_.size());
    drawMouseArrow_ = true;
}
void MapSubscriber::navigateToPoint(const geometry_msgs::PointStamped::ConstPtr& input){
//    qDebug() << "Begin";
    const double minDistance = 0.50;
    std::string source = "map";
    std::string destination = "base_link";
    try{
//        qDebug() << "point x: " << input.get()->point.x;
//        qDebug() << "point y: " << input.get()->point.y;
        geometry_msgs::PointStamped point = tfBuffer.transform(*input.get(), "map");
        geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer.lookupTransform(source, destination, ros::Time(0));

//        qDebug() << "point x: " << point.point.x;
//        qDebug() << "point y: " << point.point.y;
//        qDebug() << "pos x: " << transBaseLinkToMap.transform.translation.x;
//        qDebug() << "pos y: " << transBaseLinkToMap.transform.translation.y;

        const double x = point.point.x - transBaseLinkToMap.transform.translation.x,
                     y = point.point.y - transBaseLinkToMap.transform.translation.y;

        if(x * x + y * y < minDistance * minDistance){
          return;
        }

        const double theta = atan(y/x);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";

        pose.pose.position.x = point.point.x - minDistance * cos(theta);
        pose.pose.position.y = point.point.y - minDistance * sin(theta);

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        q.setZ(-q.z());

        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        movePub_.publish(pose);
//        qDebug() << "finished";
    }catch(...){
        qDebug() << "failed";
    }
}

void MapSubscriber::checkPointInRange(const geometry_msgs::PointStamped::ConstPtr& input){
  const double minDistance = 1.00;
  std::string source = "map";
  std::string destination = "base_link";
  try{
      geometry_msgs::PointStamped point = tfBuffer.transform(*input.get(), "map");
      geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer.lookupTransform(source, destination, ros::Time(0));

      const double x = point.point.x - transBaseLinkToMap.transform.translation.x,
                   y = point.point.y - transBaseLinkToMap.transform.translation.y;

      qDebug() << x * x + y * y;

      if(x * x + y * y < minDistance * minDistance){
        emit pointInRange(true);
        return;
      }
  }catch(...){
      qDebug() << "failed";
  }
  emit pointInRange(false);
}

MapSubscriber::~MapSubscriber() { delete tfListener; }

QPoint translateScreenToMap(QPoint p, QSize screen, QSize map) {
    return QPoint((double)p.x() * (double)map.width() / (double)screen.width(), (double)p.y() * (double)map.height() / (double)screen.height());
}

#include "mapsubscriber.hpp"

mapsubscriber::mapsubscriber(ros::NodeHandle* nodeHandle) : nh_(nodeHandle), pos_(QPoint(0,0)) {
    mapSub_ = nh_->subscribe("/map", 30, &mapsubscriber::mapCallback, this);
    posSub_ = nh_->subscribe("/stretch_diff_drive_controller/odom", 30, &mapsubscriber::posCallback, this);
    movePub_ = nh_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 30);
    tfListener = new tf2_ros::TransformListener(tfBuffer);
}

void mapsubscriber::run() {
    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();

        QPainter painter(&map_);
        painter.setCompositionMode( QPainter::CompositionMode_SourceOver );
        painter.setBrush(QBrush(QColor(Qt::red), Qt::SolidPattern));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse( pos_, 5, 5 );
        painter.setBrush(QBrush(QColor(Qt::green), Qt::SolidPattern));
        painter.drawEllipse(origin_, 5, 5);


        outputMap_ = QPixmap::fromImage(map_);

        emit mapUpdate(outputMap_);

        loop_rate.sleep();
    }
}

void mapsubscriber::mapCallback(const nav_msgs::OccupancyGrid msg) {
    const int width = msg.info.width, height = msg.info.height;
    map_ = QImage(width, height, QImage::Format_RGB888);
    resolution_ = msg.info.resolution;
    origin_ = QPoint(width + msg.info.origin.position.x/resolution_, -msg.info.origin.position.y/resolution_);

    int val = 0;
    int pos = 0;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            pos = x + msg.info.width * y;
            if (msg.data[pos] == -1) {
                val = 100;
            } else if (msg.data[pos] == 100) {
                val = 0;
            } else {
                val = 255;
            }
            map_.setPixel(width - 1 - x, y, QColor(val, val, val).rgb());
        }
    }
}

void mapsubscriber::posCallback(const nav_msgs::Odometry msg) {
    std::string source = "map";
    std::string destination = "base_link";

    try {
        geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer.lookupTransform(source, destination, ros::Time(0));

//        ROS_INFO_STREAM("x " <<  transBaseLinkToMap.transform.translation.x);
//        ROS_INFO_STREAM("y " <<  transBaseLinkToMap.transform.translation.y);

        pos_.setX(origin_.x() - transBaseLinkToMap.transform.translation.x/resolution_);
        pos_.setY(origin_.y() + transBaseLinkToMap.transform.translation.y/resolution_);
    } catch (...) {
    }
}

void mapsubscriber::moveRobot(int x, int y, int width, int height){
  geometry_msgs::PoseStamped pose;

//  ROS_INFO_STREAM("x: " << x << " y: " << y);

  x = (double)x * (double)map_.width()/(double)width;
  y = (double)y * (double)map_.height()/(double)height;


  double locX = (origin_.x() - x) * resolution_, locY = (y - origin_.y()) * resolution_;

//  ROS_INFO_STREAM("x: " << locX << " y: " << locY);
  pose.header.frame_id = "map";
  pose.pose.position.x = locX;
  pose.pose.position.y = locY;
  pose.pose.orientation.w = 1;
  movePub_.publish(pose);
}

mapsubscriber::~mapsubscriber() {
    delete tfListener;
}

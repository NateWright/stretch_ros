#include "mapsubscriber.hpp"

MapSubscriber::MapSubscriber(ros::NodeHandlePtr nodeHandle)
    : nh_(nodeHandle), robotPos_(QPoint(0, 0)), drawPos_(false), drawMouseArrow_(false) {
    mapSub_ = nh_->subscribe("/map", 30, &MapSubscriber::mapCallback, this);
    mapPointCloudSub_ = nh_->subscribe("/rtabmap/cloud_ground", 30, &MapSubscriber::mapPointCloudCallback, this);
    std::string odomTopic;
    nh_->getParam("/stretch_gui/odom", odomTopic);
    posSub_ = nh_->subscribe(odomTopic, 30, &MapSubscriber::posCallback, this);
    movePub_ = nh_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 30);
    mapPub_ = nh_->advertise<sensor_msgs::Image>("/stretch_gui/map", 30, true);
    mapColoredPub_ = nh_->advertise<sensor_msgs::Image>("/stretch_gui/map_color", 30, true);
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
    moveToThread(this);
}

MapSubscriber::~MapSubscriber() {
    spinner_->stop();
    delete spinner_;
    delete tfListener_;
}

void MapSubscriber::run() {
    spinner_ = new ros::AsyncSpinner(0);
    spinner_->start();
    exec();
}

void MapSubscriber::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    const int width = msg.get()->info.width,
              height = msg.get()->info.height;

    mapSize_.setWidth(width);
    mapSize_.setHeight(height);
    resolution_ = msg.get()->info.resolution;
    origin_ = QPoint(width + msg.get()->info.origin.position.x / resolution_, -msg.get()->info.origin.position.y / resolution_);

    // cv::Mat mapImage(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    // int val = 0;
    // for (int y = 0; y < height; y++) {
    //     for (int x = 0; x < width; x++) {
    //         cv::Vec3b color = mapImage.at<cv::Vec3b>(cv::Point(x, y));
    //         int occupancyProb = (int)msg->data[x + width * y];
    //         if (occupancyProb == -1) {
    //             val = 100;
    //         } else if (occupancyProb == 100) {
    //             val = 0;
    //         } else {
    //             val = 255;
    //         }
    //         color[0] = val;
    //         color[1] = val;
    //         color[2] = val;
    //         mapImage.at<cv::Vec3b>(cv::Point(width - 1 - x, y)) = color;
    //     }
    // }
    // mapImage_.reset(new cv_bridge::CvImage());
    // mapImage_->header = msg->header;
    // mapImage_->encoding = sensor_msgs::image_encodings::RGB8;
    // mapImage_->image = mapImage;
    // mapImageCopy_ = mapImage_;
    // mapPub_.publish(mapImage_->toImageMsg());
}

void MapSubscriber::mapPointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    if (mapSize_.width() == 0 || mapSize_.height() == 0) {
        return;
    }
    int width = mapSize_.width();
    int height = mapSize_.height();
    cv::Mat mapImage(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    cv_bridge::CvImage::Ptr map(new cv_bridge::CvImage());
    map->header.frame_id = cloud->header.frame_id;
    map->encoding = sensor_msgs::image_encodings::RGB8;
    map->image = mapImage;
    for (const auto& p : *cloud) {
        map->image.at<cv::Vec3b>(cv::Point(origin_.x() - p.x / resolution_, origin_.y() + p.y / resolution_)) = {p.r, p.g, p.b};
    }
    mapPub_.publish(map->toImageMsg());
}

void MapSubscriber::posCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::string source = "map";
    std::string destination = "base_link";

    try {
        geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer_.lookupTransform(source, destination, ros::Time(0));

        robotRot_ = tf2::getYaw(transBaseLinkToMap.transform.rotation);

        robotPos_.setX(origin_.x() - transBaseLinkToMap.transform.translation.x / resolution_);
        robotPos_.setY(origin_.y() + transBaseLinkToMap.transform.translation.y / resolution_);
        emit robotPose(robotPos_, robotRot_);

        drawPos_ = true;
    } catch (...) {
    }
}

void MapSubscriber::moveRobot(QPoint press, QPoint release, QSize screen) {
    if (press == release) {
        return;
    }
    drawMouseArrow_ = false;
    geometry_msgs::PoseStamped pose;

    QPoint mapLoc = translateScreenToMap(press, screen, mapSize_);

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

void MapSubscriber::moveRobotLoc(const geometry_msgs::PoseStamped::Ptr pose) {
    movePub_.publish(pose);
}
void MapSubscriber::navigateToPoint(const geometry_msgs::PointStamped::ConstPtr& input) {
    //    qDebug() << "Begin";
    const double minDistance = 0.50;
    std::string source = "map";
    std::string destination = "base_link";
    try {
        //        qDebug() << "point x: " << input.get()->point.x;
        //        qDebug() << "point y: " << input.get()->point.y;
        geometry_msgs::PointStamped point = tfBuffer_.transform(*input.get(), "map");
        geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer_.lookupTransform(source, destination, ros::Time(0));

        //        qDebug() << "point x: " << point.point.x;
        //        qDebug() << "point y: " << point.point.y;
        //        qDebug() << "pos x: " << transBaseLinkToMap.transform.translation.x;
        //        qDebug() << "pos y: " << transBaseLinkToMap.transform.translation.y;

        const double x = point.point.x - transBaseLinkToMap.transform.translation.x,
                     y = point.point.y - transBaseLinkToMap.transform.translation.y;

        if (x * x + y * y < minDistance * minDistance) {
            return;
        }

        const double theta = atan(y / x);

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
    } catch (...) {
        qDebug() << "failed";
    }
}

void MapSubscriber::checkPointInRange(const geometry_msgs::PointStamped::ConstPtr& input) {
    const double minDistance = 1.00;
    std::string source = "map";
    std::string destination = "base_link";
    try {
        geometry_msgs::PointStamped point = tfBuffer_.transform(*input.get(), "map");
        geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer_.lookupTransform(source, destination, ros::Time(0));

        const double x = point.point.x - transBaseLinkToMap.transform.translation.x,
                     y = point.point.y - transBaseLinkToMap.transform.translation.y;

        //      qDebug() << x * x + y * y;
        //      qDebug() << "x: " << point.point.x;
        //      qDebug() << "y: " << point.point.y;
        //      qDebug() << "z: " << point.point.z;

        if (x * x + y * y < minDistance * minDistance) {
            emit validPoint();
            return;
        }
    } catch (...) {
        qDebug() << "failed";
    }
    emit invalidPoint();
}

void MapSubscriber::setHome() {
    std::string source = "map",
                destination = "base_link";
    geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer_.lookupTransform(source, destination, ros::Time(0));

    robotHome_.header.frame_id = transBaseLinkToMap.header.frame_id;
    robotHome_.pose.orientation = transBaseLinkToMap.transform.rotation;
    robotHome_.pose.position.x = transBaseLinkToMap.transform.translation.x;
    robotHome_.pose.position.y = transBaseLinkToMap.transform.translation.y;
    robotHome_.pose.position.z = transBaseLinkToMap.transform.translation.z;
    emit homeSet(true);
}

void MapSubscriber::setHomeIfNone() {
    std::string s = robotHome_.header.frame_id;
    if (s.length() == 0) {
        setHome();
    }
}

void MapSubscriber::navigateHome() {
    movePub_.publish(robotHome_);
}
void MapSubscriber::disableMapping() {
    std_srvs::Empty msg;
    ros::service::call("/rtabmap/pause", msg);
}
void MapSubscriber::enableMapping() {
    std_srvs::Empty msg;
    ros::service::call("/rtabmap/resume", msg);
}

void MapSubscriber::rotate(int degrees) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    tf2::Quaternion q;
    q.setRPY(0, 0, degrees * M_PI / 180);
    pose.pose.orientation = tf2::toMsg(q);
    movePub_.publish(pose);
}

void MapSubscriber::rotateLeft(int degrees) {
    rotate(degrees);
}
void MapSubscriber::rotateRight(int degrees) {
    rotate(-degrees);
}

void MapSubscriber::drive(double meters) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = meters;
    pose.pose.orientation.w = 1;
    movePub_.publish(pose);
}

QPoint translateScreenToMap(QPoint p, QSize screen, QSize map) {
    return QPoint((double)p.x() * (double)map.width() / (double)screen.width(), (double)p.y() * (double)map.height() / (double)screen.height());
}

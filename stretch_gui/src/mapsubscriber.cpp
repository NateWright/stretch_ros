#include "mapsubscriber.hpp"

MapSubscriber::MapSubscriber(ros::NodeHandlePtr nodeHandle)
    : nh_(nodeHandle), robotPos_(QPoint(0, 0)), drawPos_(false), drawMouseArrow_(false) {
    mapSub_ = nh_->subscribe("/map", 30, &MapSubscriber::mapCallback, this);
    std::string odomTopic;
    nh_->getParam("/stretch_gui/odom", odomTopic);
    posSub_ = nh_->subscribe(odomTopic, 30, &MapSubscriber::posCallback, this);
    movePub_ = nh_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 30);
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
    map_ = QImage(10, 10, QImage::Format_RGB888);
    moveToThread(this);
}

MapSubscriber::~MapSubscriber() { delete tfListener_; }

void MapSubscriber::run() {
    QTimer* timer = new QTimer();
    timer->setInterval(15);
    connect(timer, &QTimer::timeout, this, &MapSubscriber::loop);
    timer->start();
    exec();
    delete timer;
}

void MapSubscriber::loop() {
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
        geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer_.lookupTransform(source, destination, ros::Time(0));

        robotRot_ = tf2::getYaw(transBaseLinkToMap.transform.rotation);

        robotPos_.setX(origin_.x() - transBaseLinkToMap.transform.translation.x / resolution_);
        robotPos_.setY(origin_.y() + transBaseLinkToMap.transform.translation.y / resolution_);

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

void MapSubscriber::moveRobotLoc(const geometry_msgs::PoseStamped::Ptr pose) {
    movePub_.publish(pose);
}

void MapSubscriber::mousePressInitiated(QPoint press, QSize screen) {
    mousePressLocation_ = translateScreenToMap(press, screen, map_.size());
}

void MapSubscriber::mousePressCurrentLocation(QPoint loc, QSize screen) {
    mousePressCurrentLocation_ = translateScreenToMap(loc, screen, map_.size());
    drawMouseArrow_ = true;
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
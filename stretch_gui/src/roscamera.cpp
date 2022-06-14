#include "roscamera.hpp"

RosCamera::RosCamera(ros::NodeHandle* nh) : nh_(nh), showCenterPoint_(false) {
    //     colorCameraSub_ = nh_->subscribe("/camera/depth/color/points", 30, &RosCamera::cameraCallback, this);
    colorCameraSub_ = nh_->subscribe("/camera/depth_registered/points", 30, &RosCamera::cameraCallback, this);
    //    colorCameraSub_ = nh_->subscribe("/stretch_pc/pointcloud", 30, &RosCamera::cameraCallback, this);
    pointPick_ = nh->advertise<geometry_msgs::PointStamped>("/clicked_point", 30);
    centerPointSub_ = nh_->subscribe("/stretch_pc/centerPoint", 30, &RosCamera::centerPointCallback, this);
    moveToThread(this);
}
RosCamera::~RosCamera() {}

void RosCamera::run() {
    QTimer* timer = new QTimer();
    timer->setInterval(15);
    connect(timer, &QTimer::timeout, this, &RosCamera::loop);
    timer->start();
    exec();
    delete timer;
}

void RosCamera::loop() {
    ros::spinOnce();

    QImage img = cameraOutputRotated_.toImage();
    if (showCenterPoint_) {
        QPainter painter(&img);
        painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
        painter.setBrush(QBrush(QColor(Qt::red), Qt::SolidPattern));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(centerPoint_, 20, 20);
        painter.end();
    }
    cameraOutputRotatedWithPoint_ = QPixmap::fromImage(img);
    emit imgUpdateWithPoint(cameraOutputRotatedWithPoint_);
    emit imgUpdate(cameraOutputRotated_);
}

void RosCamera::cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pc) {
    cloud_ = pc;

    const int width = pc->width,
              height = pc->height;
    camera_ = QImage(height, width, QImage::Format_RGB888);

    pcl::PointXYZRGB point;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            point = pc->at(x, y);
            camera_.setPixel(height - 1 - y, x, QColor(point.r, point.g, point.b).rgb());
        }
    }
    cameraOutputRotated_ = QPixmap::fromImage(camera_);
    //    cameraOutputRotated_ = cameraOutput_.transformed(QTransform().rotate(90));
}

void RosCamera::centerPointCallback(const geometry_msgs::PointStamped::ConstPtr& point) {
    pcl::PointXYZ p1, curr;
    p1 = pcl::PointXYZ(point->point.x, point->point.y, 0);
    float smallest = NAN;
    float currDistance = 0;

    const int width = cloud_->width,
              height = cloud_->height;
    QPoint min(height, 0);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            curr = pcl::PointXYZ(cloud_->at(x, y).x, cloud_->at(x, y).y, 0);
            currDistance = pcl::euclideanDistance(p1, curr);
            if (std::isnan(smallest)) {
                min = QPoint(height - y, x);
                smallest = currDistance;
                continue;
            }
            if (currDistance < smallest) {
                min = QPoint(height - y, x);
                smallest = currDistance;
            }
        }
    }
    centerPoint_ = min;
    emit checkPointInRange(point);
}

void RosCamera::sceneClicked(QPoint press, QPoint release, QSize screen) {
    //  int locX = press.x() * (cloud_->width / screen.width());
    //  int locY = press.y() * (cloud_->height / screen.height());
    int locX = press.x(),
        locY = press.y();

    geometry_msgs::PointStamped point;
    point.header.frame_id = cloud_->header.frame_id;

    try {
        if (locY > cloud_->width || locX > cloud_->height) {
            throw(std::runtime_error("Not in range"));
        }
        emit clickInitiated();
        pcl::PointXYZRGB p = cloud_->at(locY, cloud_->height - locX);

        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
            qDebug() << "click fail";
            emit clickFailure();
            throw(std::runtime_error("Point contains NaN"));
        }

        point.point.x = p.x;
        point.point.y = p.y;
        point.point.z = p.z;

        pointPick_.publish(point);
        emit clickSuccess();
    } catch (...) {
    }
}
void RosCamera::showCenterPoint() {
    showCenterPoint_ = true;
}
void RosCamera::hideCenterPoint() {
    showCenterPoint_ = false;
}

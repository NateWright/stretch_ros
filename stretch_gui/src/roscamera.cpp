#include "roscamera.hpp"

RosCamera::RosCamera(ros::NodeHandlePtr nh) : nh_(nh) {
    std::string pointCloudTopic;
    nh->getParam("/stretch_gui/pointCloudTopic", pointCloudTopic);
    colorCameraSub_ = nh_->subscribe(pointCloudTopic, 30, &RosCamera::cameraCallback, this);
    segmentedCameraSub_ = nh_->subscribe("/stretch_pc/cluster", 30, &RosCamera::segmentedCameraCallback, this);
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
    emit imgUpdate(cameraOutputRotated_);
}

void RosCamera::segmentedCameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pc) {
    const int width = cloud_->width,
              height = cloud_->height;
    QImage img = cameraOutputRotated_.toImage();

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud_);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    QRgb red = QColor(Qt::red).rgb();
    for (pcl::PointXYZRGB p : *pc) {
        kdtree.nearestKSearch(p, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        int pos = pointIdxNKNSearch[0];
        img.setPixel(height - 1 - pos / width, pos % width, red);
    }
    cameraOutputRotatedWithPoint_ = QPixmap::fromImage(img);
    emit imgUpdateWithPoint(cameraOutputRotatedWithPoint_);
}

void RosCamera::centerPointCallback(const geometry_msgs::PointStamped::ConstPtr& point) {
    emit checkPointInRange(point);
    return;
}

void RosCamera::sceneClicked(QPoint press, QPoint release, QSize screen) {
    //  int locX = press.x() * (cloud_->width / screen.width());
    //  int locY = press.y() * (cloud_->height / screen.height());
    //    int locX = press.x(),
    //        locY = static_cast<double>(press.y()) * static_cast<double>(cloud_->width) / static_cast<double>(screen.height());
    int locX = press.x(),
        locY = press.y();

    try {
        if (locY > cloud_->width || locX > cloud_->height) {
            throw(std::runtime_error("Not in range"));
        }
        emit clickInitiated();
        pcl::PointXYZRGB p = cloud_->at(locY, cloud_->height - locX);

        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
            qDebug() << "click fail";
            throw(std::runtime_error("Point contains NaN"));
        }

        geometry_msgs::PointStamped point;
        point.header.frame_id = cloud_->header.frame_id;

        point.point.x = p.x;
        point.point.y = p.y;
        point.point.z = p.z;

        pointPick_.publish(point);
        emit clickSuccess();
    } catch (...) {
        emit clickFailure();
        return;
    }
}
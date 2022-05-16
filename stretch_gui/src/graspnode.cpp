#include "graspnode.hpp"

GraspNode::GraspNode(ros::NodeHandle *nh) : nh_(nh) {
    resetPub_ = nh_->advertise<std_msgs::Bool>("/stretch_pc/reset", 1);
    itemSub_ = nh_->subscribe("/stretch_pc/pointcloud", 30,
                              &GraspNode::itemCloudCallback, this);
}

GraspNode::~GraspNode() {}

void GraspNode::run() {
    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        emit imgUpdate(cameraOutputRotated_);

        loop_rate.sleep();
    }
}

void GraspNode::itemCloudCallback(const sensor_msgs::PointCloud2 pc) {
    camera_ = QImage(pc.width, pc.height, QImage::Format_RGB888);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pc, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    pcl::PointXYZRGB point;

    for (auto p = cloud.get()->begin(); p != cloud.get()->end(); p++) {
        p->x
    }
    for (int y = 0; y < pc.height; y++) {
        for (int x = 0; x < pc.width; x++) {
            point = cloud.get()->at(x, y);
            camera_.setPixel(x, y, QColor(point.r, point.g, point.b).rgb());
        }
    }
    cameraOutput_ = QPixmap::fromImage(camera_);
    cameraOutputRotated_ = cameraOutput_.transformed(QTransform().rotate(90));
}

void GraspNode::reset() {
    std_msgs::Bool b;
    b.data = 1;
    resetPub_.publish(b);
}

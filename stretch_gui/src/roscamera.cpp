#include "roscamera.hpp"

RosCamera::RosCamera(ros::NodeHandle *nh) : nh_(nh) {
    colorCameraSub_ = nh_->subscribe("/camera/depth/color/points", 30,
                                     &RosCamera::cameraCallback, this);
    cameraAdjustment_ =
        nh_->advertise<stretch_moveit_grasps::stretch_move_bool>("/move_head",
                                                                 30);
    pointPick_ =
        nh->advertise<geometry_msgs::PointStamped>("/clicked_point", 30);
}
RosCamera::~RosCamera() {}

void RosCamera::run() {
    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        emit imgUpdate(cameraOutputRotated_);

        loop_rate.sleep();
    }
}

void RosCamera::cameraCallback(const sensor_msgs::PointCloud2 pc) {
    frameId_ = pc.header.frame_id;
    camera_ = QImage(pc.width, pc.height, QImage::Format_RGB888);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pc, pcl_pc2);

    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    pcl::PointXYZRGB point;
    for (int y = 0; y < pc.height; y++) {
        for (int x = 0; x < pc.width; x++) {
            point = cloud.get()->at(x, y);
            camera_.setPixel(x, y, QColor(point.r, point.g, point.b).rgb());
        }
    }
    cameraOutput_ = QPixmap::fromImage(camera_);
    cameraOutputRotated_ = cameraOutput_.transformed(QTransform().rotate(90));
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

void RosCamera::sceneClicked(int x, int y, int width, int height) {
    int locX = (double)x * (double)camera_.width() / (double)width;
    int locY = (double)x * (double)camera_.height() / (double)height;

    geometry_msgs::PointStamped point;
    point.header.frame_id = frameId_;

    pcl::PointXYZRGB p = cloud.get()->at(locX, locY);

    point.point.x = p.x;
    point.point.y = p.y;
    point.point.z = p.z;

    pointPick_.publish(point);
}

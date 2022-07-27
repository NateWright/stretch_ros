#include "graspnode.hpp"

GraspNode::GraspNode(ros::NodeHandlePtr nh) : nh_(nh) {
    resetPub_ = nh_->advertise<std_msgs::Bool>("/stretch_pc/reset", 30);
    // cmdVelPub_ = nh_->advertise<geometry_msgs::Twist>("/stretch_diff_drive_controller/cmd_vel", 1000);
    cmdArmPub_ = nh_->advertise<geometry_msgs::Pose>("/stretch_moveit_grasps/arm", 1000);
    centerPointSub_ = nh_->subscribe("/stretch_pc/centerPoint", 30, &GraspNode::centerPointCallback, this);

    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
    point_.reset(new geometry_msgs::PointStamped());
    moveToThread(this);
}

GraspNode::~GraspNode() {
    spinner_->stop();
    delete spinner_;
    delete tfListener_;
}

void GraspNode::run() {
    spinner_ = new ros::AsyncSpinner(0);
    spinner_->start();
    exec();
}

void GraspNode::centerPointCallback(const geometry_msgs::PointStamped::ConstPtr& input) {
    geometry_msgs::PointStamped point = tfBuffer_.transform(*input.get(), "map");
    point_.reset(new geometry_msgs::PointStamped());
    *point_.get() = point;
    point = tfBuffer_.transform(*input.get(), "base_link");
    pointBaseLink_.reset(new geometry_msgs::PointStamped());
    *pointBaseLink_.get() = point;
}

void GraspNode::lineUp() {
    switch (orientation) {
        case VERTICAL: {
            lineUpOffset(0.33);
            break;
        }
        case HORIZONTAL: {
            lineUpOffset(0.36);
            break;
        }
    }
}

void GraspNode::setHorizontal() {
    orientation = HORIZONTAL;
}
void GraspNode::setVertical() {
    orientation = VERTICAL;
}
void GraspNode::lineUpOffset(double offset) {
    ros::AsyncSpinner s(1);
    s.start();
    ros::Duration d(0.5);
    emit disableMapping();
    std::string targetFrame = "map", sourceFrame = "base_link";

    geometry_msgs::TransformStamped transBaseToMap = tfBuffer_.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
    homePose_.reset(new geometry_msgs::PoseStamped());
    homePose_->header.frame_id = "map";
    homePose_->pose.position.x = transBaseToMap.transform.translation.x;
    homePose_->pose.position.y = transBaseToMap.transform.translation.y;
    homePose_->pose.position.z = transBaseToMap.transform.translation.z;
    homePose_->pose.orientation = transBaseToMap.transform.rotation;

    geometry_msgs::PoseStamped::Ptr pose(new geometry_msgs::PoseStamped());

    pose->header.frame_id = "base_link";
    tf2::Quaternion q;
    q.setRPY(0, 0, atan(pointBaseLink_->point.y / pointBaseLink_->point.x) + 96 * M_PI / 180);
    pose->pose.orientation = tf2::toMsg(q);
    emit navigate(pose);
    d.sleep();

    d.sleep();
    emit headSetPan(-90);
    d.sleep();
    emit armSetHeight(pointBaseLink_->point.z + 0.05);
    d.sleep();
    emit gripperSetRotate(0);
    d.sleep();
    emit gripperSetGrip(30);
    d.sleep();
    emit armSetHeight(pointBaseLink_->point.z);
    d.sleep();
    emit armSetReach(sqrt(pointBaseLink_->point.x * pointBaseLink_->point.x + pointBaseLink_->point.y * pointBaseLink_->point.y) - offset);
    emit graspDone(true);
    emit canNavigate(false);
}

void GraspNode::replaceObject() {
    ros::AsyncSpinner s(1);
    s.start();
    ros::Duration d(0.5);
    emit disableMapping();

    emit navigate(homePose_);

    geometry_msgs::PoseStamped::Ptr pose(new geometry_msgs::PoseStamped());

    pose->header.frame_id = "base_link";
    tf2::Quaternion q;
    q.setRPY(0, 0, atan(pointBaseLink_->point.y / pointBaseLink_->point.x) + 96 * M_PI / 180);
    pose->pose.orientation = tf2::toMsg(q);
    emit navigate(pose);
    d.sleep();

    d.sleep();
    emit headSetPan(-90);
    d.sleep();
    emit armSetHeight(pointBaseLink_->point.z + 0.05);
    d.sleep();
    emit gripperSetRotate(0);
    d.sleep();
    emit armSetHeight(pointBaseLink_->point.z);
    d.sleep();
    emit armSetReach(sqrt(pointBaseLink_->point.x * pointBaseLink_->point.x + pointBaseLink_->point.y * pointBaseLink_->point.y) - 0.36);
    d.sleep();
    emit gripperSetGrip(30);
    d.sleep();
    emit armSetHeight(pointBaseLink_->point.z + 0.05);
    emit hasObject(false);
}

void GraspNode::releaseObject() {
    ros::Duration d(3.0);
    emit armSetHeight(0.5);
    d.sleep();
    emit gripperSetGrip(30);
    emit hasObject(false);
}

void GraspNode::stowObject() {
    emit hasObject(true);
    ros::Duration d(1.0);
    emit gripperSetGrip(-3);
    d.sleep();
    d.sleep();
    emit armSetHeight(pointBaseLink_->point.z + 0.05);
    d.sleep();
    emit gripperSetRotate(90);
    d.sleep();
    emit armSetReach();
    d.sleep();
    emit headSetTilt(-30);
    d.sleep();
    emit headSetPan();
    d.sleep();
    emit armSetHeight(0.40);
    d.sleep();
    emit enableMapping();
    emit canNavigate(true);
}

void GraspNode::home() {
    ros::Duration d(0.25);
    //  emit armSetHeight(1.05);
    d.sleep();
    emit armSetReach();
    d.sleep();
    emit headSetPan();
    d.sleep();
    emit gripperSetGrip();
    d.sleep();
    emit gripperSetRotate();
    d.sleep();
    emit armSetHeight();
    d.sleep();
    emit enableMapping();
    d.sleep();
    d.sleep();
    emit navigate(homePose_);
    emit canNavigate(true);
}
#include "ObjectSegmenter.hpp"

ObjectSegmenter::ObjectSegmenter(ros::NodeHandlePtr nh) : nh_(nh) {
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
    clusterPub_ = nh_->advertise<sensor_msgs::PointCloud2>("/stretch_pc/cluster", 1000);
    pointPub_ = nh_->advertise<geometry_msgs::PointStamped>("/stretch_pc/centerPoint", 1000);
}

void ObjectSegmenter::segmentAndFind(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloud, int x, int y) {
    segmentation(pointCloud);
    findCluster(x, y);
}

// void ObjectSegmenter::segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc) {
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pc;

//     pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
//     pcl::IndicesPtr indices(new std::vector<int>);
//     pcl::removeNaNFromPointCloud(*cloud, *indices);

//     pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
//     reg.setInputCloud(cloud);
//     reg.setIndices(indices);
//     reg.setSearchMethod(tree);
//     reg.setDistanceThreshold(10);
//     reg.setPointColorThreshold(6);
//     reg.setRegionColorThreshold(5);
//     reg.setMinClusterSize(600);

//     reg.extract(clusters_);

//     colored_cloud_ = reg.getColoredCloud();

//     colored_cloud_->header.frame_id = pc->header.frame_id;

//     return;
// }

void ObjectSegmenter::segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pc;

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(cloud);
    reg.setIndices(indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    reg.extract(clusters_);

    colored_cloud_ = reg.getColoredCloud();

    colored_cloud_->header.frame_id = pc->header.frame_id;

    return;
}

void ObjectSegmenter::findCluster(int posX, int posY) {
    ROS_INFO_STREAM("Picking event occurred");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster;

    ROS_INFO_STREAM("Looking for cluster");
    bool done = false;

    int pos = posY * colored_cloud_->width + posX;

    for (pcl::PointIndices p : clusters_) {
        cloud_cluster.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& idx : p.indices) {
            cloud_cluster->push_back((*colored_cloud_)[idx]);
            if (idx == pos) {
                cloud_cluster->width = cloud_cluster->size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
                done = true;
            }
        }
        if (done) {
            ROS_INFO_STREAM("Cluster found");
            break;
        }
    }

    float x = 0,
          y = 0,
          z = 0;
    int count = 0;
    for (const auto& p : cloud_cluster->points) {
        x += p.x;
        y += p.y;
        z += p.z;
        count++;
    }

    geometry_msgs::PointStamped pStamped;
    pStamped.point.x = x / count;
    pStamped.point.y = y / count;
    pStamped.point.z = z / count;
    pStamped.header.frame_id = colored_cloud_->header.frame_id;
    pointPub_.publish(pStamped);

    cloud_cluster->header.frame_id = colored_cloud_->header.frame_id;
    clusterPub_.publish(cloud_cluster);
}
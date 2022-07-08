#include "ObjectSegmenter.hpp"

ObjectSegmenter::ObjectSegmenter(ros::NodeHandlePtr nh) : nh_(nh) {
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
    clusterPub_ = nh_->advertise<sensor_msgs::PointCloud2>("/stretch_pc/cluster", 1000);
    pointPub_ = nh_->advertise<geometry_msgs::PointStamped>("/stretch_pc/centerPoint", 1000);
}

void ObjectSegmenter::segmentAndFind(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloud, const geometry_msgs::PointStamped::ConstPtr pointStamped) {
    segmentation(pointCloud);
    pointPickingEventOccurred(pointStamped);
}

void ObjectSegmenter::segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pc;

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(cloud);
    reg.setIndices(indices);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(10);
    reg.setPointColorThreshold(6);
    reg.setRegionColorThreshold(5);
    reg.setMinClusterSize(600);

    reg.extract(clusters_);

    colored_cloud_ = reg.getColoredCloud();

    colored_cloud_->header.frame_id = pc->header.frame_id;

    return;
}

void ObjectSegmenter::pointPickingEventOccurred(const geometry_msgs::PointStamped::ConstPtr inputPoint) {
    ROS_INFO_STREAM("Picking event occurred");

    geometry_msgs::PointStamped tfPoint = tfBuffer_.transform(*inputPoint, colored_cloud_->header.frame_id);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointXYZRGB p;
    p.x = tfPoint.point.x;
    p.y = tfPoint.point.y;
    p.z = tfPoint.point.z;

    ROS_INFO_STREAM("Looking for cluster");
    bool done = false;

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(colored_cloud_);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    kdtree.nearestKSearch(p, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    int pos = pointIdxNKNSearch[0];

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
    for (auto p : cloud_cluster->points) {
        x += p.x;
        y += p.y;
        z += p.z;
        count++;
    }

    geometry_msgs::PointStamped pStamped;
    pStamped.point.x = x / count;
    pStamped.point.y = y / count;
    pStamped.point.z = z / count;
    pStamped.header = tfPoint.header;
    pointPub_.publish(pStamped);

    cloud_cluster->header.frame_id = colored_cloud_->header.frame_id;
    clusterPub_.publish(cloud_cluster);
}
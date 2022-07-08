#include <pcl/common/distances.h>
// #include <pcl/conversions.h>
#include <pcl/filters/filter_indices.h>  // for pcl::removeNaNFromPointCloud
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <thread>
#include <vector>

#pragma once

using namespace std::chrono_literals;

class ObjectSegmenter {
   private:
    // Setup
    ros::NodeHandlePtr nh_;

    // Publishers and Subscribers
    ros::Publisher clusterPub_;
    ros::Publisher pointPub_;

    // Indicies and Main Cloud
    std::vector<pcl::PointIndices> clusters_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_;

    // Frames for publishing
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener* tfListener_;

   public:
    explicit ObjectSegmenter(ros::NodeHandlePtr nh);
    ~ObjectSegmenter() {
        delete tfListener_;
    }
    void segmentAndFind(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, const geometry_msgs::PointStamped::ConstPtr);
    void segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
    void pointPickingEventOccurred(const geometry_msgs::PointStamped::ConstPtr);
};

typedef std::shared_ptr<ObjectSegmenter> ObjectSegmenterPtr;

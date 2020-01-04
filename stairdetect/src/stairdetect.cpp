#include "stairdetect/stairdetect.h"

stairDetector::stairDetector(ros::NodeHandle &n, const std::string &s, int bufSize)
{
    _subPointCloud = n.subscribe(
        "/velodyne_points", 50, & stairDetector::callback_pointcloud, this
    );
}

void stairDetector::callback_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & msg)
{
    ROS_INFO("in pointcloud callback...");

    // stitch pointclouds
    // etc 

    return;
}
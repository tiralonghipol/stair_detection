#include "stairdetect/stairdetect.h"

stairDetector::stairDetector(ros::NodeHandle &n, const std::string &s, int bufSize)
{
    n.getParam("topic_stitched_pointcloud", _topic_stitched_pcl);
    // _topic_stitched_pcl = 
    // if(n.getParam("topic_stitched_pointcloud", _topic_stitched_pcl)){
    //     ROS_INFO("Topic")
    // }


    _sub_stitched_pcl = n.subscribe(
        _topic_stitched_pcl, 50, & stairDetector::callback_stitched_pcl, this
    );
}

void stairDetector::callback_stitched_pcl(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & msg)
{
    ROS_INFO("in stitched pointcloud callback...");

    // stitch pointclouds
    // etc 
    return;
}
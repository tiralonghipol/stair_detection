#ifndef STAIRDETECTOR_H_
#define STAIRDETECTOR_H_

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"
#include <pcl/registration/icp.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <string>
#include <cmath>
// #include <Eigen/Dense>
#include "Eigen/Geometry"
#include <ctime>

// opencv stuff
#include <opencv2/core/core.hpp>
#include "opencv2/core/mat.hpp"
#include <cv_bridge/cv_bridge.h>


using namespace std;
class stairDetector{
    public:
    stairDetector(ros::NodeHandle & n, const std::string & s, int bufSize);
    void callback_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & msg);

    private:
    ros::Subscriber _subPointCloud;
};

#endif 

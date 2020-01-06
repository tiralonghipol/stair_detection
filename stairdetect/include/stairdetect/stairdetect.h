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
#include <pcl_ros/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <string>
#include <cmath>
#include <ctime>
// #include <Eigen/Dense>
#include "Eigen/Geometry"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// opencv stuff
#include <opencv2/core/core.hpp>
#include "opencv2/core/mat.hpp"
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>


using namespace std;
class stairDetector{
    public:
    stairDetector(ros::NodeHandle & n, const std::string & s, int bufSize);
    
    // callbacks
    // void callback_stitched_pcl(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & msg);
    void callback_stitched_pcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr & msg);
    void callback_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg);

    // other functions
    void trim_stitched_pcl(pcl::PCLPointCloud2 & trimmed_cloud);
    // cv::Mat pcl2bird_view()

    void filter_img(const cv::Mat &bird_view_img);
  	// void houghLine(const cv::Mat &edge_image, Lines &lines);



    private:
    // subscribers
    ros::Subscriber _sub_stitched_pcl;
    ros::Subscriber _sub_pose;

    // publishers
    ros::Publisher _pub_trimmed_pcl;

    // topic names
    std::string _topic_stitched_pcl;
    std::string _topic_pose;
    std::string _topic_trimmed_pcl;  

    // pointcloud trimming params
    int _xy_lim;
    int _z_lim;

    // test mode flag
    bool _test_mode;

    // 
    int _pose_Q_size;
    std::deque<geometry_msgs::PoseWithCovarianceStamped> _pose_Q;
};

#endif 

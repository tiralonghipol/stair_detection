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
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <dynamic_reconfigure/server.h>
#include <stairdetect/StairDetectConfig.h>

using namespace std;
using namespace cv;

struct stairDetectorParams
{
  bool debug;
  double canny_low_th = 40;
  double canny_ratio = 10;
  int canny_kernel_size = 1;

  // pcl to img params
  // THESE SHOULD PROBABLY COME FROM LIDAR_STITCH_PARAMS
  double img_xy_dim = 25;
  double img_resolution = 0.02;
};

class stairDetector
{
public:
  stairDetector(ros::NodeHandle &n, const std::string &s, int bufSize);
  void setParam(const stairDetectorParams &param);

  // callbacks
  // void callback_stitched_pcl(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & msg);
  void callback_stitched_pcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg);
  void callback_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void callback_timer_trigger(const ros::TimerEvent &event);

  // other functions
  void trim_stitched_pcl(pcl::PCLPointCloud2 &trimmed_cloud);
  void pcl_to_bird_view(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg);

  // cv::Mat pcl2bird_view()

  // image operations
  // void filter_img(const cv::Mat &bird_view_img);
  void pcl_to_bird_view_img(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, cv::Mat &img);
  void filter_img();
  void cannyEdgeDetection(const cv::Mat &input_image, cv::Mat &edge);

  // void houghLine(const cv::Mat &edge_image, Lines &lines);

private:
  // subscribers
  ros::Subscriber _sub_stitched_pcl;
  ros::Subscriber _sub_pose;

  // publishers
  ros::Publisher _pub_trimmed_pcl;
  image_transport::Publisher _pub_bird_view_img;

  // timers
  ros::Timer _timer_stair_detect;

  // topic names
  std::string _topic_stitched_pcl;
  std::string _topic_pose;
  std::string _topic_trimmed_pcl;
  std::string _topic_bird_eye_img;

  // pointcloud trimming params
  int _xy_lim;
  int _z_lim;
  double _stair_detect_time;

  // reference to most recently-received cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr _recent_cloud;

  // parameter container
  stairDetectorParams _param;

  // dynamic reconfigure
  // dynamic_reconfigure::Server<stairdetect::StairDetectConfig> _dr_server;

  // pose queue
  int _pose_Q_size;
  std::deque<geometry_msgs::PoseWithCovarianceStamped> _pose_Q;
};

#endif

#ifndef STAIRDETECTOR_H_
#define STAIRDETECTOR_H_

#include "ros/ros.h"
#include "stairdetect/line.h"
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
  // canny
  double canny_low_th = 50;
  double canny_ratio = 50;
  int canny_kernel_size = 1;

  // morphological filter
  int morph_kernel_size = 3;
  int morph_num_iter = 1;

  // line segment detection
  // https://codeutils.xyz/OpenCV3.3.0/dd/d1a/group__imgproc__feature.html#ga6b2ad2353c337c42551b521a73eeae7d
  double lsd_scale = 0.8;
  double lsd_sigma_scale = 0.6;
  double lsd_quant = 2.0;
  double lsd_angle_th = 22.5;
  double lsd_log_eps = 0.0;
  double lsd_density_th = 0.1;
  int lsd_n_bins = 1024;

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
  void callback_stitched_pcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg);
  void callback_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void callback_timer_trigger(const ros::TimerEvent &event);
  void callback_dyn_reconf(stairdetect::StairDetectConfig &config, uint32_t level);

  // other functions
  void trim_stitched_pcl(pcl::PCLPointCloud2 &trimmed_cloud);
  void pcl_to_bird_view(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg);

  // image operations
  void pcl_to_bird_view_img(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, cv::Mat &img);
  void filter_img(cv::Mat &img);
  void canny_edge_detect(const cv::Mat &input_image, cv::Mat &edge);
  void morph_filter(const cv::Mat &img_in, cv::Mat &img_out);
  void skeleton_filter(const cv::Mat &img_in, cv::Mat &img_out);
  void lsd_lines(const cv::Mat &img_in, Lines &lines);
  void draw_lines(cv::Mat &image, const Lines &lines, const cv::Scalar &color);
  void publish_img_msgs(cv::Mat &img_bird_view, cv::Mat &img_proc, cv::Mat &img_line, cv::Mat &img_line_filtered);

  void cluster_by_kmeans(const cv::Mat &img, Lines &lines, vector<Lines> clustered_lines);

private:
  // subscribers
  ros::Subscriber _sub_stitched_pcl;
  ros::Subscriber _sub_pose;

  // publishers
  ros::Publisher _pub_trimmed_pcl;
  image_transport::Publisher _pub_bird_view_img;
  image_transport::Publisher _pub_proc_img;
  image_transport::Publisher _pub_morph_img;
  image_transport::Publisher _pub_line_img;
  image_transport::Publisher _pub_filtered_line_img;

  // timers
  ros::Timer _timer_stair_detect;

  // topic names
  std::string _topic_stitched_pcl;
  std::string _topic_pose;
  std::string _topic_trimmed_pcl;
  std::string _topic_bird_eye_img;
  std::string _topic_proc_img;
  std::string _topic_line_img;
  std::string _topic_filtered_line_img;

  // pointcloud trimming params
  double _stair_detect_time;

  // reference to most recently-received cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr _recent_cloud;

  // parameter container
  stairDetectorParams _param;

  // dynamic reconfigure
  dynamic_reconfigure::Server<stairdetect::StairDetectConfig> _dr_srv;
  dynamic_reconfigure::Server<stairdetect::StairDetectConfig>::CallbackType _dyn_rec_cb;

  // pose queue
  int _pose_Q_size;
  std::deque<geometry_msgs::PoseWithCovarianceStamped> _pose_Q;
};

#endif

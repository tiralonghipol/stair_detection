#ifndef STAIRDETECTOR_H_
#define STAIRDETECTOR_H_

#include "ros/ros.h"
#include "stairdetect/line.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <string>
#include <cmath>
#include <ctime>
// #include <Eigen/Dense>
#include "Eigen/Geometry"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>

// opencv stuff
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ml.hpp>

#include <opencv2/ximgproc.hpp>
#include <opencv2/photo.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <stairdetect/StairDetectConfig.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace cv;
using namespace cv::ml;

struct stairDetectorParams
{
  bool debug;

  // morphological filter
  int morph_kernel_size = 2;
  int morph_num_iter = 2;

  int median_kernel_size = 3;

  // line segment detection
  // https://codeutils.xyz/OpenCV3.3.0/dd/d1a/group__imgproc__feature.html#ga6b2ad2353c337c42551b521a73eeae7d
  double lsd_scale = 0.25;
  double lsd_sigma_scale = 0.25;
  double lsd_quant = 2.0;
  double lsd_angle_th = 22.5;
  double lsd_log_eps = 0.0;
  double lsd_density_th = 0.1;
  int lsd_n_bins = 1024;

  // pcl to img params
  // THESE SHOULD PROBABLY COME FROM LIDAR_STITCH_PARAMS
  double img_xy_dim = 25;
  double img_resolution = 0.02;

  double staircase_max_area = 10.0;
  double staircase_min_area = 1.5;
};

class stairDetector
{
public:
  stairDetector(ros::NodeHandle &n, const std::string &s, int bufSize);
  void set_param(const stairDetectorParams &param);

  // callbacks
  void callback_stitched_pcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg);
  void callback_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void callback_timer_trigger(const ros::TimerEvent &event);
  void callback_dyn_reconf(stairdetect::StairDetectConfig &config, uint32_t level);

  // image operations
  void pcl_to_bird_view_img(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, cv::Mat &img);
  vector<vector<vector<cv::Point>>> filter_img(cv::Mat &img);
  void morph_filter(const cv::Mat &img_in, cv::Mat &img_out);
  void lsd_lines(const cv::Mat &img_in, Lines &lines);

  // visualization
  void draw_lines(cv::Mat &image, const Lines &lines, const cv::Scalar &color);
  void draw_clustered_lines(cv::Mat &image, const vector<Lines> &clustered_lines);

  // publish
  void publish_img_msgs(cv::Mat &img_bird_view, cv::Mat &img_proc, cv::Mat &img_line, cv::Mat &img_line_filtered);

  // clustering
  void cluster_by_kmeans(const cv::Mat &img, Lines &lines, vector<Lines> &clustered_lines);

  // filtering
  void process_clustered_lines(const vector<Lines> &clustered_lines, vector<Lines> &processed_lines);
  vector<Lines> subcluster_by_orientation(const vector<Lines> &clustered_lines);
  vector<Lines> filter_lines_by_angle(const Lines &lines_in);
  Lines filter_lines_by_mid_pts_dist(const Lines &lines_in);
  Lines filter_lines_by_gransac(const Lines &lines_in);

  vector<vector<cv::Point>> calc_cluster_bounds(const Lines &lines);
  vector<vector<Eigen::Vector3d>> hull_px_to_wf(
      const vector<vector<vector<cv::Point>>> & hulls);

  Eigen::Vector2d px_to_m(const cv::Point &pt);
  geometry_msgs::PolygonStamped hull_to_polygon_msg(const vector<Eigen::Vector3d> &hull, int seq_id);
  visualization_msgs::Marker centroid_to_marker_msg(const Eigen::Vector2d & centroid);
  geometry_msgs::PoseStamped hull_centroid_to_pose_msg(
    const Eigen::Vector2d & centroid);

  vector<Eigen::Vector2d> hull_centroid_px_to_wf(
    const vector<cv::Point> centroids_px);

  void process_and_publish_centroids(
      const vector<Eigen::Vector2d> & hull_centroids_wf);


  bool check_hull_area(const double &hull_area);

  // clusters colors
  Scalar random_color(RNG &rng);

  cv::Point calc_centroid_pixel(const vector<cv::Point> &hull);

private:
  // subscribers
  ros::Subscriber _sub_stitched_pcl;
  ros::Subscriber _sub_pose;

  // keep track of transform when stitched cloud is received
  tf::TransformListener _tf_listener;

  // publishers
  ros::Publisher _pub_trimmed_pcl;
  ros::Publisher _pub_staircase_polygon;
  ros::Publisher _pub_centroid_marker;
  ros::Publisher _pub_centroid_pose;
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
  std::string _topic_polygon;
  std::string _topic_centroid_vis;
  std::string _topic_centroid_pose;

  // frame names
  std::string _frame_world;
  std::string _frame_pcl;

  // pointcloud trimming params
  double _stair_detect_time;

  // reference to most recently-received cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr _recent_cloud;
  geometry_msgs::PoseWithCovarianceStamped _recent_pose;
  tf::StampedTransform _recent_tf;
  geometry_msgs::PoseWithCovarianceStamped _callback_pose;

  // parameter container
  stairDetectorParams _param;

  // dynamic reconfigure
  dynamic_reconfigure::Server<stairdetect::StairDetectConfig> _dr_srv;
  dynamic_reconfigure::Server<stairdetect::StairDetectConfig>::CallbackType _dyn_rec_cb;

  // pose queue
  int _pose_Q_size;
  std::deque<geometry_msgs::PoseWithCovarianceStamped> _pose_Q;

  // color generation
  vector<Scalar> _color_tab;

  // stair parameters
  int _max_clusters;
  int _min_stair_steps = 3;

  // check if stair is already detected
  // vector<Point> _total_centroids;
  vector<Eigen::Vector2d> _confirmed_centroids;
  vector<Eigen::Vector2d> _unconfirmed_centroids;
  int _min_dist_between_stairs = 1.5;
};

#endif

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

// #define PI 3.141592654

using namespace std;
using namespace cv;

// class Line
// {
// public:
//   // In opencv x is col, y is row

//   Line(double x1 = 0, double y1 = 0, double x2 = 0, double y2 = 0)
//   {
//     this->k = (y1 - y2) / (x1 - x2);
//     this->b = y1 - k * x1;
//     // this->t = - atan2( y1 - y2, x1 - x2);
//     this->t = atan((y1 - y2) / (x1 - x2));
//     this->r = x1 * sin(t) + y1 * cos(t);
//     this->p1 = *new cv::Point(x1, y1);
//     this->p2 = *new cv::Point(x2, y2);
//     this->center = p1 - p2;
//     this->length = cv::norm(this->p1 - this->p2);
//   }

//   Line(cv::Point p1, cv::Point p2) : Line(p1.x, p1.y, p2.x, p2.y) {}

//   Line(cv::Vec4i line) : Line(line[0], line[1], line[2], line[3]) {}

//   Line() {}

//   void calPixels(cv::Mat src)
//   {
//     this->pixels.clear();
//     cv::LineIterator it(src, p1, p2, 8, true);
//     pixels_num = it.count;
//     for (int i = 0; i < pixels_num; i++, ++it)
//     {
//       pixels.push_back(it.pos());
//     }
//   }

//   friend std::ostream &operator<<(std::ostream &os, const Line &line)
//   {
//     os << "Point1: " << line.p1 << " Point2: " << line.p2 << " k:" << line.k
//        << " b:" << line.b << " t: " << line.t << " r: " << line.r;
//     return os;
//   }

//   double k, b = 0;
//   double r, t = 0;
//   double length = 0;
//   cv::Point p1;
//   cv::Point p2;
//   cv::Point center;
//   int pixels_num = 0;
//   // cv::LineIterator it;
//   std::vector<cv::Point> pixels;
// };
// typedef std::vector<Line> Lines;

struct stairDetectorParams
{
  bool debug;
  // canny
  double canny_low_th = 10;
  double canny_ratio = 20;
  int canny_kernel_size = 1;
  // hough transfrom
  double hough_min_line_length = 50; // pixel distance
  double hough_max_line_gap = 15;    // pixel distance
  double hough_th = 20;
  int hough_rho = 8;
  double hough_theta = 1; // angle

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
  void callback_dyn_reconf(stairdetect::StairDetectConfig &config, uint32_t level);

  // other functions
  void trim_stitched_pcl(pcl::PCLPointCloud2 &trimmed_cloud);
  void pcl_to_bird_view(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg);

  // cv::Mat pcl2bird_view()

  // image operations
  // void filter_img(const cv::Mat &bird_view_img);
  void pcl_to_bird_view_img(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, cv::Mat &img);
  void filter_img(cv::Mat &img);
  void canny_edge_detect(const cv::Mat &input_image, cv::Mat &edge);
  void hough_lines(const cv::Mat &edge_image, Lines &lines);
  void draw_lines(cv::Mat &image, const Lines &lines, const cv::Scalar &color);

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
  dynamic_reconfigure::Server<stairdetect::StairDetectConfig> _dr_srv;
  dynamic_reconfigure::Server<stairdetect::StairDetectConfig>::CallbackType _dyn_rec_cb;


  // pose queue
  int _pose_Q_size;
  std::deque<geometry_msgs::PoseWithCovarianceStamped> _pose_Q;
};

#endif

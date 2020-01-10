#ifndef LINE_H_
#define LINE_H_

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

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

#define PI 3.141592654

using namespace std;
using namespace cv;

class Line
{
public:
  // In opencv x is col, y is row

  Line(double x1 = 0, double y1 = 0, double x2 = 0, double y2 = 0)
  {
    this->k = (y1 - y2) / (x1 - x2);
    this->b = y1 - k * x1;
    // this->t = - atan2( y1 - y2, x1 - x2);
    this->t = atan((y1 - y2) / (x1 - x2));
    this->r = x1 * sin(t) + y1 * cos(t);
    this->p1 = *new cv::Point(x1, y1);
    this->p2 = *new cv::Point(x2, y2);
    this->p_mid.x = (p1.x + p2.x) / 2;
    this->p_mid.y = (p1.y + p2.y) / 2;
    this->length = cv::norm(this->p1 - this->p2);
  }

  Line(cv::Point p1, cv::Point p2) : Line(p1.x, p1.y, p2.x, p2.y) {}

  Line(cv::Vec4i line) : Line(line[0], line[1], line[2], line[3]) {}

  Line() {}

  void calPixels(cv::Mat src)
  {
    this->pixels.clear();
    cv::LineIterator it(src, p1, p2, 8, true);
    pixels_num = it.count;
    for (int i = 0; i < pixels_num; i++, ++it)
    {
      pixels.push_back(it.pos());
    }
  }

  friend std::ostream &operator<<(std::ostream &os, const Line &line)
  {
    os << "P1: " << line.p1
       << "\tP2: " << line.p2 
       << "\tk: " << line.k
       << "\tid: " << line.cluster_id << "\n";
    // os << "\nPoint1: " << line.p1 << " Point2: " << line.p2 << " k:" << line.k
    //    << " b:" << line.b << " t: " << line.t << " r: " << line.r << "\n";
    return os;
  }

  double k, b = 0;
  double r, t = 0;
  double length = 0;
  cv::Point p1;
  cv::Point p2;
  cv::Point2f p_mid;
  int pixels_num = 0;
  int cluster_id = 0;
  double p_mid_z = 0;
  // cv::LineIterator it;
  std::vector<cv::Point> pixels;
};
typedef std::vector<Line> Lines;

#endif

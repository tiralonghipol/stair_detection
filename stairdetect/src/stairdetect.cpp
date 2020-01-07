#include "stairdetect/stairdetect.h"
#include "stairdetect/line.h"

stairDetector::stairDetector(ros::NodeHandle &n, const std::string &s, int bufSize)
{
    // _recent_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    _recent_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    // dynamic reconfigure
    // _dyn_rec_cb = boost::bind(&stairDetector::callback_dyn_reconf, this, _1, _2);
    // _dr_srv.setCallback(_dyn_rec_cb);

    // get params from .launch file
    // topics
    n.getParam("topic_stitched_pointcloud", _topic_stitched_pcl);
    n.getParam("topic_pose", _topic_pose);
    n.getParam("topic_trimmed_pcl", _topic_trimmed_pcl);
    n.getParam("topic_bird_eye_img", _topic_bird_eye_img);
    n.getParam("topic_edge_img", _topic_edge_img);
    n.getParam("topic_line_img", _topic_line_img);
    n.getParam("stair_detect_timing", _stair_detect_time);

    // pointcloud trimming parameters
    _xy_lim = 10;
    _z_lim = 1;

    // other params
    _pose_Q_size = 40;

    // test flag
    n.getParam("debug", _param.debug);

    // subscribers
    _sub_stitched_pcl = n.subscribe(
        _topic_stitched_pcl, 1, &stairDetector::callback_stitched_pcl, this);
    _sub_pose = n.subscribe(
        _topic_pose, 50, &stairDetector::callback_pose, this);

    // publishers
    _pub_trimmed_pcl = n.advertise<sensor_msgs::PointCloud2>(_topic_trimmed_pcl, bufSize);
    image_transport::ImageTransport it_0(n);
    _pub_bird_view_img = it_0.advertise(_topic_bird_eye_img, 1);
    image_transport::ImageTransport it_1(n);
    _pub_edge_img = it_1.advertise(_topic_edge_img, 1);
    image_transport::ImageTransport it_2(n);
    _pub_line_img = it_2.advertise(_topic_line_img, 1);

    // timers
    _timer_stair_detect = n.createTimer(
        ros::Duration(_stair_detect_time), &stairDetector::callback_timer_trigger, this);

    // // dynamic reconfigure
    // _dyn_rec_cb = boost::bind(&stairDetector::callback_dyn_reconf, this, _1, _2);
    // _dr_srv.setCallback(_dyn_rec_cb);

    setParam(_param);
}

void stairDetector::callback_stitched_pcl(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
{
    _recent_cloud = msg;
    return;
}

void stairDetector::callback_timer_trigger(
    const ros::TimerEvent &event)
{
    cv::Mat img(
        int(_param.img_xy_dim / _param.img_resolution),
        int(_param.img_xy_dim / _param.img_resolution),
        CV_8UC1,
        cv::Scalar(0));

    // convert most recently-received pointcloud to birds-eye-view image
    pcl_to_bird_view_img(_recent_cloud, img);

    // filtering on birds-eye image
    filter_img(img);
    return;
}

void stairDetector::callback_dyn_reconf(stairdetect::StairDetectConfig &config, uint32_t level)
{
    // high level bools
    // _stairdetect_config = config;
    // stairDetectorParams new_params;

    _param.debug = config.debug;
    // canny
    _param.canny_low_th = config.canny_low_th;
    _param.canny_ratio = config.canny_ratio;
    _param.canny_kernel_size = config.canny_kernel_size;
    // hough transform
    _param.hough_min_line_length = config.hough_min_line_length;
    _param.hough_max_line_gap = config.hough_max_line_gap;
    _param.hough_th = config.hough_th;
    _param.hough_rho = config.hough_rho;
    _param.hough_theta = config.hough_theta;

    setParam(_param);
}

void stairDetector::pcl_to_bird_view_img(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    cv::Mat &img)
{
    int img_midpt = int(_param.img_xy_dim / _param.img_resolution) / 2;

    // get current xy
    double x_0 = _pose_Q[0].pose.pose.position.x;
    double y_0 = _pose_Q[0].pose.pose.position.y;

    // build birds-eye-view image
    int idx_x = 0;
    int idx_y = 0;
    for (const pcl::PointXYZ pt : cloud->points)
    {
        idx_x = int((x_0 - pt.x) / _param.img_resolution) + img_midpt;
        idx_y = int((y_0 - pt.y) / _param.img_resolution) + img_midpt;
        // NOTE: check which order to use
        // img.at<uchar>(idx_y, idx_x)
        img.at<uchar>(idx_x, idx_y) = 255;
    }
    return;
}

void stairDetector::trim_stitched_pcl(
    pcl::PCLPointCloud2 &trimmed_cloud)
{
    return;
}

// keep track of pose, used for trimming pointcloud. _pose_Q[0] contains most recent pose
void stairDetector::callback_pose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    // dereference
    geometry_msgs::PoseWithCovarianceStamped msg_in = std::remove_const<std::remove_reference<decltype(*msg)>::type>::type(*msg);

    // update queue
    _pose_Q.push_front(msg_in);
    if (_pose_Q.size() > _pose_Q_size)
    {
        _pose_Q.pop_back();
    }
    return;
}

void stairDetector::setParam(const stairDetectorParams &param)
{
    _param = param;
    _param.hough_theta = _param.hough_theta * PI / 180;

    _param.canny_kernel_size = _param.canny_kernel_size * 2 + 1;
    if (_param.canny_kernel_size > 7)
    {
        _param.canny_kernel_size = 7;
    }
    if (_param.canny_kernel_size < 1)
    {
        _param.canny_kernel_size = 1;
    }
    return;
}

// void filter_img(const cv::Mat &bird_view_img)
void stairDetector::filter_img(cv::Mat &img)
{
    Mat edge_img, hough_img;

    // edge detection
    canny_edge_detect(img, edge_img);

    // line detection
    Lines lines;
    hough_lines(edge_img, lines);
    cv::cvtColor(edge_img, hough_img, CV_GRAY2BGR);
    draw_lines(hough_img, lines, cv::Scalar(255, 0, 0));

    if (_param.debug)
    {
        publish_img_msgs(img, edge_img, hough_img);
    }
    return;
}

void stairDetector::publish_img_msgs(
    cv::Mat &img_bird_view,
    cv::Mat &img_edge,
    cv::Mat &img_line)
{
    // publish birds-eye image
    sensor_msgs::ImagePtr bird_view_img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_bird_view).toImageMsg();
    _pub_bird_view_img.publish(bird_view_img_msg);

    // publish edge image
    sensor_msgs::ImagePtr edge_img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_edge).toImageMsg();
    _pub_edge_img.publish(edge_img_msg);

    // publish line (hugh) image
    sensor_msgs::ImagePtr line_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_line).toImageMsg();
    _pub_line_img.publish(line_img_msg);
    return;
}

void stairDetector::canny_edge_detect(const cv::Mat &input_image, cv::Mat &edge)
{
    /// Reduce noise with a kernel 3x3
    cv::blur(input_image, edge, cv::Size(3, 3));
    /// Canny detector
    cv::Canny(edge, edge, (double)_param.canny_low_th, (double)_param.canny_low_th * _param.canny_ratio, _param.canny_kernel_size);
    return;
}

void stairDetector::hough_lines(const cv::Mat &edge_image, Lines &lines)
{
    lines.clear();
    if (_param.hough_theta == 0)
    {
        _param.hough_theta = 1 * PI / 180;
    }
    if (_param.hough_rho == 0)
    {
        _param.hough_rho = 1;
    }

    std::vector<cv::Vec4i> xy_lines;
    HoughLinesP(edge_image, xy_lines,
                (double)_param.hough_rho / 10,
                (double)_param.hough_theta,
                (int)_param.hough_th,
                (double)_param.hough_min_line_length,
                (double)_param.hough_max_line_gap);

    for (int i = 0; i < xy_lines.size(); i++)
    {
        Line line(xy_lines[i]);
        line.calPixels(edge_image);
        lines.push_back(line);
    }
}

void stairDetector::draw_lines(cv::Mat &image, const Lines &lines, const cv::Scalar &color)
{
    for (int i = 0; i < lines.size(); i++)
    {
        // std::cout << "Drwing lines: " << lines[i] << std::endl;
        cv::line(image, lines[i].p1, lines[i].p2, color, 3, 8);
        // putText(image, std::to_string(lines[i].t), lines[i].p1, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 0));
    }
}
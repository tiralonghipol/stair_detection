#include "stairdetect/stairdetect.h"
#include "stairdetect/line.h"

stairDetector::stairDetector(ros::NodeHandle &n, const std::string &s, int bufSize)
{
    _recent_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // get params from .launch file
    // topics
    n.getParam("topic_stitched_pointcloud", _topic_stitched_pcl);
    n.getParam("topic_pose", _topic_pose);
    n.getParam("topic_trimmed_pcl", _topic_trimmed_pcl);
    n.getParam("topic_bird_eye_img", _topic_bird_eye_img);
    n.getParam("topic_edge_img", _topic_edge_img);
    n.getParam("topic_line_img", _topic_line_img);
    n.getParam("stair_detect_timing", _stair_detect_time);
    n.getParam("line_detection_method", _line_detection_method);

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

    // dynamic reconfigure
    _dyn_rec_cb = boost::bind(&stairDetector::callback_dyn_reconf, this, _1, _2);
    _dr_srv.setCallback(_dyn_rec_cb);

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
    ROS_INFO("Reconfigure Triggered");
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
    // lsd
    _param.lsd_scale = config.lsd_scale;
    _param.lsd_sigma_scale = config.lsd_sigma_scale;
    _param.lsd_quant = config.lsd_quant;
    _param.lsd_angle_th = config.lsd_angle_th;
    _param.lsd_log_eps = config.lsd_log_eps;
    _param.lsd_density_th = config.lsd_density_th;

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
    // Mat edge_img, hough_img;
    Mat edge_img, line_img, filtered_line_img;

    // edge detection
    canny_edge_detect(img, edge_img);

    // line detection
    Lines lines;
    if (_line_detection_method == "hough")
    {
        hough_lines(edge_img, lines);
    }
    else if (_line_detection_method == "lsd")
    {
        lsd_lines(edge_img, lines);
    }
    else
    {
        ROS_WARN("LINE DETECTION ALGORITHM SPECIFIED DOES NOT EXIST (must be hough or lsd)");
    }

    // plot lines on image
    cv::cvtColor(edge_img, line_img, CV_GRAY2BGR);
    cv::cvtColor(edge_img, filtered_line_img, CV_GRAY2BGR);
    draw_lines(line_img, lines, cv::Scalar(255, 0, 0));

    Lines filtered_lines;
    // filter_lines_by_slope_hist(lines, filtered_lines);

    // draw_lines(filtered_line_img, filtered_lines, cv::Scalar(255, 0, 0));

    if (_param.debug)
    {
        publish_img_msgs(img, edge_img, line_img);
        // publish_img_msgs(img, line_img, filtered_line_img);
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

    // publish line image
    sensor_msgs::ImagePtr line_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_line).toImageMsg();
    _pub_line_img.publish(line_img_msg);
    return;
}

void stairDetector::canny_edge_detect(const cv::Mat &input_image, cv::Mat &edge)
{
    /// Reduce noise with a kernel 3x3
    // cv::blur(input_image, edge, cv::Size(3, 3));
    cv::medianBlur(input_image, edge, 3);
    /// Canny detector
    cv::Canny(edge, edge, (double)_param.canny_low_th, (double)_param.canny_low_th * _param.canny_ratio, _param.canny_kernel_size);
    return;
}

void stairDetector::draw_lines(cv::Mat &image, const Lines &lines, const cv::Scalar &color)
{
    for (int i = 0; i < lines.size(); i++)
    {
        cv::line(image, lines[i].p1, lines[i].p2, color, 1, 8);
    }
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
    return;
}

void stairDetector::lsd_lines(const cv::Mat &img_in, Lines &lines)
{
    vector<Vec4i> xy_lines;
    Ptr<LineSegmentDetector> lsd = createLineSegmentDetector(
        LSD_REFINE_STD,
        _param.lsd_scale,
        _param.lsd_sigma_scale,
        _param.lsd_quant,
        _param.lsd_angle_th,
        _param.lsd_log_eps,
        _param.lsd_density_th,
        _param.lsd_n_bins);
    lsd->detect(img_in, xy_lines);

    for (int i = 0; i < xy_lines.size(); i++)
    {
        Line line(xy_lines[i]);
        line.calPixels(img_in);
        lines.push_back(line);
    }
    return;
}

void stairDetector::filter_lines_by_slope_hist(const Lines &input_lines, Lines &filtered_lines)
{
    // calculate histogram
    std::vector<int> slope_hist;
    std::vector<std::vector<int>> slope_hist_list;
    // 10 degree
    double bin_width = _param.filter_slope_hist_bin_width;
    int bin_num = PI / bin_width;

    // initialize the histogram
    for (int i = 0; i < bin_num; i++)
    {
        slope_hist.push_back(0);
        std::vector<int> empty;
        slope_hist_list.push_back(empty);
    }

    // calculate slope_hist value
    for (int i = 0; i < input_lines.size(); i++)
    {
        double t = input_lines[i].t;
        if (t < 0)
        {
            t = t + PI;
        }
        if (t > PI)
        {
            t = t - PI;
        }
        // std::cout << xy_linesk << std::endl;
        int id = t / bin_width;
        if (id > bin_num)
        {
            id = 0;
        }
        if (id < 0)
        {
            id = bin_num;
        }
        slope_hist[id]++;
        slope_hist_list[id].push_back(i);
    }

    // print histogram value
    // for (int i = 0; i < slope_hist.size(); i++) {
    // 	std::cout << slope_hist[i] << std::endl;
    // }

    // calcuate the maximum frequency angle
    int max_id = std::distance(slope_hist.begin(), std::max_element(slope_hist.begin(), slope_hist.end()));
    if (_param.debug)
    {
        std::cout << "Maximum frequency is between angle: "
                  << max_id * bin_width * 180 / PI << " and "
                  << (max_id + 1) * bin_width * 180 / PI << std::endl;
    }

    // std::cout << "Which has totoal line: " << slope_hist_list[max_id].size() <<
    // std::endl;
    // extract the filtered lines

    // Lines filtered_lines;
    for (int i = 0; i < slope_hist_list[max_id].size(); i++)
    {
        int id = slope_hist_list[max_id][i];
        filtered_lines.push_back(input_lines[id]);
    }
    return;
}

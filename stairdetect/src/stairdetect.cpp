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
    n.getParam("topic_proc_img", _topic_proc_img);
    n.getParam("topic_line_img", _topic_line_img);
    n.getParam("topic_filtered_line_img", _topic_filtered_line_img);

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

    boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedPtr;

    while (sharedPtr == NULL)
    {
        ROS_INFO("Stitched PCL not received yet");
        sharedPtr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(_topic_stitched_pcl, ros::Duration(2));
    }

    // publishers
    _pub_trimmed_pcl = n.advertise<sensor_msgs::PointCloud2>(_topic_trimmed_pcl, bufSize);
    image_transport::ImageTransport it_0(n);
    _pub_bird_view_img = it_0.advertise(_topic_bird_eye_img, 1);
    image_transport::ImageTransport it_1(n);
    _pub_proc_img = it_1.advertise(_topic_proc_img, 1);
    image_transport::ImageTransport it_2(n);
    _pub_line_img = it_2.advertise(_topic_line_img, 1);
    image_transport::ImageTransport it_3(n);
    _pub_filtered_line_img = it_3.advertise(_topic_filtered_line_img, 1);
    // show final result of filtering on this topic

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
    Mat img(
        int(_param.img_xy_dim / _param.img_resolution),
        int(_param.img_xy_dim / _param.img_resolution),
        CV_8UC1,
        Scalar(0));

    // convert most recently-received pointcloud to birds-eye-view image
    pcl_to_bird_view_img(_recent_cloud, img);

    // filtering on birds-eye image
    filter_img(img);
    return;
}

void stairDetector::callback_dyn_reconf(stairdetect::StairDetectConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Triggered");
    // high level bools
    _param.debug = config.debug;
    // canny
    _param.canny_low_th = config.canny_low_th;
    _param.canny_ratio = config.canny_ratio;
    _param.canny_kernel_size = config.canny_kernel_size;
    // morphological
    _param.morph_kernel_size = config.morph_kernel_size;
    _param.morph_num_iter = config.morph_num_iter;

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
    // filter stuff
    _param.filter_slope_hist_bin_width = config.filter_slope_hist_bin_width;

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;

    pcl::PointXYZ minPt, maxPt;
    // pcl::getMinMax3D
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
    _param.filter_slope_hist_bin_width = _param.filter_slope_hist_bin_width * PI / 180;
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

void stairDetector::filter_img(cv::Mat &raw_img)
{
    Mat proc_img, line_img, filtered_line_img;

    // edge detection
    // canny_edge_detect(img, edge_img);
    // img = imread("/home/pol/stair_ws/src/stairdetect/imgs_test/top_view_matlab.pgm", CV_LOAD_IMAGE_GRAYSCALE);
    // instead of using canny, use just a binary image
    // imshow("original", img);
    // waitKey(30);

    // threshold(img, edge_img, 0, 255, THRESH_BINARY);
    // imshow("after threshold", edge_img);
    // waitKey(30);

    morph_filter(raw_img, proc_img);

    blur(proc_img, proc_img, Size(3, 3));
    // bilateralFilter(edge_img, blurred_img, 3, 500, 250);
    // medianBlur(edge_img, edge_img, 3);
    // imshow("after edge detection", edge_img);
    // waitKey(30);
    // cvtColor(edge_img, edge_img, CV_BINA);
    // imshow("after skel", edge_img);
    // waitKey(30);

    // line detection
    Lines lines;
    if (_line_detection_method == "hough")
    {
        hough_lines(proc_img, lines);
    }
    else if (_line_detection_method == "lsd")
    {
        lsd_lines(proc_img, lines);
    }
    else
    {
        ROS_WARN("LINE DETECTION ALGORITHM SPECIFIED DOES NOT EXIST (must be hough or lsd)");
    }
    if (!lines.empty())
    {

        // plot lines on image
        cvtColor(proc_img, line_img, CV_GRAY2BGR);
        draw_lines(line_img, lines, Scalar(255, 0, 0));

        // line clustering
        Lines filtered_lines;
        // filter_lines_by_slope_hist(lines, filtered_lines);
        cluster_by_kmeans(line_img, lines);
        draw_lines(filtered_line_img, filtered_lines, Scalar(0, 255, 0));

        if (_param.debug)
        {
            publish_img_msgs(raw_img, proc_img, line_img, filtered_line_img);
        }
    }
    else
    {
        ROS_WARN("Not enough lines detected");
    }

    return;
}

void stairDetector::publish_img_msgs(
    cv::Mat &img_bird_view,
    cv::Mat &img_proc,
    cv::Mat &img_line,
    cv::Mat &img_line_filtered)
{
    if (_pub_bird_view_img.getNumSubscribers() > 0)
    {
        // publish birds-eye image
        sensor_msgs::ImagePtr bird_view_img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_bird_view).toImageMsg();
        _pub_bird_view_img.publish(bird_view_img_msg);
    }
    if (_pub_proc_img.getNumSubscribers() > 0)
    {
        // publish edge image
        sensor_msgs::ImagePtr proc_img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_proc).toImageMsg();
        _pub_proc_img.publish(proc_img_msg);
    }
    if (_pub_line_img.getNumSubscribers() > 0)
    {
        // publish line image
        sensor_msgs::ImagePtr line_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_line).toImageMsg();
        _pub_line_img.publish(line_img_msg);
    }
    if (_pub_filtered_line_img.getNumSubscribers() > 0)
    {
        sensor_msgs::ImagePtr filtered_line_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_line_filtered).toImageMsg();
        _pub_filtered_line_img.publish(filtered_line_img_msg);
    }
    return;
}

void stairDetector::canny_edge_detect(const cv::Mat &input_image, cv::Mat &edge)
{
    /// Reduce noise with a kernel 3x3
    blur(input_image, edge, Size(3, 3));
    /// Canny detector
    Canny(edge, edge, (double)_param.canny_low_th, (double)_param.canny_low_th * _param.canny_ratio, _param.canny_kernel_size);
    return;
}

void stairDetector::morph_filter(const cv::Mat &img_in, cv::Mat &img_out)
{
    // erosion, dilation filters
    // https://docs.opencv.org/trunk/d4/d86/group__imgproc__filter.html#gaeb1e0c1033e3f6b891a25d0511362aeb
    erode(
        img_in,
        img_out,
        // Mat(),
        Mat(_param.morph_kernel_size, _param.morph_kernel_size, CV_8UC1),
        Point(-1, -1),
        _param.morph_num_iter,
        1,
        1);
    dilate(
        img_out,
        img_out,
        // Mat(),
        Mat(_param.morph_kernel_size, _param.morph_kernel_size, CV_8UC1),
        Point(-1, -1),
        _param.morph_num_iter,
        1,
        1);
    return;
}

void stairDetector::draw_lines(cv::Mat &image, const Lines &lines, const cv::Scalar &color)
{
    for (int i = 0; i < lines.size(); i++)
    {
        line(image, lines[i].p1, lines[i].p2, color, 2, 8);
    }
}

void stairDetector::hough_lines(const cv::Mat &proc_img, Lines &lines)
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

    std::vector<Vec4i> xy_lines;
    HoughLinesP(proc_img, xy_lines,
                (double)_param.hough_rho / 10,
                (double)_param.hough_theta,
                (int)_param.hough_th,
                (double)_param.hough_min_line_length,
                (double)_param.hough_max_line_gap);

    for (int i = 0; i < xy_lines.size(); i++)
    {
        Line line(xy_lines[i]);
        line.calPixels(proc_img);
        lines.push_back(line);
    }
    ROS_INFO_ONCE("Found %d lines", lines.size());
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

    // calcuate the maximum frequency angle
    int max_id = std::distance(slope_hist.begin(), std::max_element(slope_hist.begin(), slope_hist.end()));
    if (_param.debug)
    {
        ROS_INFO("Maximum frequency is between angle: %f and %f ",
                 max_id * bin_width * 180 / PI,
                 (max_id + 1) * bin_width * 180 / PI);
    }

    // extract the filtered lines
    for (int i = 0; i < slope_hist_list[max_id].size(); i++)
    {
        int id = slope_hist_list[max_id][i];
        filtered_lines.push_back(input_lines[id]);
    }
    ROS_INFO_ONCE("Input: %d | Output: %d lines",
                  input_lines.size(),
                  filtered_lines.size());

    return;
}

void stairDetector::cluster_by_kmeans(const cv::Mat &img, Lines &lines)
{
    int i;
    Mat points, labels, centers;

    const int MAX_CLUSTERS = 5;
    Scalar colorTab[] =
        {
            Scalar(0, 0, 255),
            Scalar(0, 255, 0),
            Scalar(255, 100, 100),
            Scalar(255, 0, 255),
            Scalar(0, 255, 255)};

    // Mat bg_img(320, 240, CV_8UC3, Scalar(0, 0, 0));

    std::vector<cv::Point2f> mid_pts;

    for (i = 0; i < lines.size(); i++)
    {
        points.push_back(lines[i].p_mid);
        circle(img, lines[i].p_mid, 10, Scalar(0, 255, 0), 1, 8);
    }

    double compactness = kmeans(points, MAX_CLUSTERS, labels,
                                TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 100, 1.0),
                                3, KMEANS_PP_CENTERS, centers);

    // imshow("mid points", img);
    // waitKey(30);
    // cout << m_mid_pts << endl;
    // cout << "-----------------" << endl;
    // cout << labels << endl;
    // cout << "-----------------" << endl;
    // cout << centers << endl;
    // cout << "-----------------" << endl;

    // img = Scalar::all(0);
    for (i = 0; i < lines.size(); i++)
    {
        int clusterIdx = labels.at<int>(i);
        Point ipt = points.at<Point2f>(i);
        circle(img, ipt, 2, colorTab[clusterIdx], FILLED, LINE_AA);
    }
    for (i = 0; i < centers.rows; ++i)
    {
        Point2f c = centers.at<Point2f>(i);
        circle(img, c, 40, colorTab[i], 1, LINE_AA);
    }
    // cout << "Compactness: " << compactness << endl;
    // imshow("clusters", img);
    // waitKey(30);

    return;
}
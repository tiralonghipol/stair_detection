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

    // dynamic reconfigure
    _dyn_rec_cb = boost::bind(&stairDetector::callback_dyn_reconf, this, _1, _2);
    _dr_srv.setCallback(_dyn_rec_cb);

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

    setParam(_param);
}

void stairDetector::callback_stitched_pcl(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
{
    if (!msg->empty())
    {
        _recent_cloud = msg;
    }
    else
    {
        ROS_INFO("Message is empty");
    }
    return;
}

void stairDetector::callback_timer_trigger(
    const ros::TimerEvent &event)
{
    ROS_INFO("in timing trigger callback");
    Mat img(
        int(_param.img_xy_dim / _param.img_resolution),
        int(_param.img_xy_dim / _param.img_resolution),
        CV_8UC1,
        Scalar(0));

    if (!_recent_cloud->empty())
    {
        // convert most recently-received pointcloud to birds-eye-view image
        pcl_to_bird_view_img(_recent_cloud, img);
        // filtering on birds-eye image
        filter_img(img);
    }
    else
    {
        ROS_INFO("No stitch in progress");
    }

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

    // lsd
    _param.lsd_scale = config.lsd_scale;
    _param.lsd_sigma_scale = config.lsd_sigma_scale;
    _param.lsd_quant = config.lsd_quant;
    _param.lsd_angle_th = config.lsd_angle_th;
    _param.lsd_log_eps = config.lsd_log_eps;
    _param.lsd_density_th = config.lsd_density_th;

    setParam(_param);
    return;
}

void stairDetector::pcl_to_bird_view_img(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    cv::Mat &img)
{
    ROS_INFO("in pcl_to_bird_view_img");
    int img_midpt = int(_param.img_xy_dim / _param.img_resolution) / 2;

    // get current xy
    double x_0 = _pose_Q[0].pose.pose.position.x;
    double y_0 = _pose_Q[0].pose.pose.position.y;

    // build birds-eye-view image
    int idx_x = 0;
    int idx_y = 0;
    for (const pcl::PointXYZ pt : cloud->points)
    {
        /*
        NOTE:
        these image coordinates may be reversed. may need to switch in order
        to correctly extract real-world coordinates from pixel coordinates
        */
        idx_x = int((x_0 - pt.x) / _param.img_resolution) + img_midpt;
        idx_y = int((y_0 - pt.y) / _param.img_resolution) + img_midpt;

        /* 
        must check that pixel index lies with image, as robot may have moved since
        the stitched pointcloud was constructed
        */
        if (!(idx_x >= img.size().width || idx_y >= img.size().height) && !(idx_x < 0 || idx_y < 0))
        {
            img.at<uchar>(idx_x, idx_y) = 255;
        }
    }
    return;
}

void stairDetector::trim_stitched_pcl(
    pcl::PCLPointCloud2 &trimmed_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;

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
    ROS_INFO("IN filter_img");
    // Mat proc_img, line_img, filtered_line_img;
    Mat line_img, filtered_line_img;

    Mat proc_img(raw_img.size(), CV_8UC1);

    // edge detection
    // canny_edge_detect(img, edge_img);
    // img = imread("/home/pol/stair_ws/src/stairdetect/imgs_test/top_view_matlab.pgm", CV_LOAD_IMAGE_GRAYSCALE);
    // instead of using canny, use just a binary image
    // imshow("original", img);
    // waitKey(30);

    // Mat t_img;
    // threshold(raw_img, t_img, 0, 255, THRESH_BINARY);
    // imshow("after threshold", edge_img);
    // waitKey(30);

    // Mat morph_img(raw_img.size(), CV_8UC1);
    // morph_filter(raw_img, morph_img);
    // Mat t_img;
    // threshold(morph_img, t_img, 0, 255, THRESH_BINARY);
    // skeleton_filter(morph_img, proc_img);
    // skeleton_filter(t_img, proc_img);
    // threshold(proc_img, proc_img, 0, 255, THRESH_BINARY);

    // ---------------------------------------- //
    morph_filter(raw_img, proc_img);

    // bilateralFilter(edge_img, blurred_img, 3, 500, 250);
    // medianBlur(edge_img, edge_img, 3);
    // imshow("after edge detection", edge_img);
    // waitKey(30);
    // cvtColor(edge_img, edge_img, CV_BINA);
    // imshow("after skel", edge_img);
    // waitKey(30);

    // line detection
    Lines lines;
    lsd_lines(proc_img, lines);

    if (lines.size() > 3)
    {
        // plot lines on image
        cvtColor(proc_img, line_img, CV_GRAY2BGR);
        draw_lines(line_img, lines, Scalar(255, 0, 0));

        // line clustering
        vector<Lines> clustered_lines;

        cluster_by_kmeans(line_img, lines, clustered_lines);
        // draw_lines(filtered_line_img, filtered_lines, Scalar(0, 255, 0));

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

void stairDetector::skeleton_filter(
    const cv::Mat &img_in,
    cv::Mat &img_out)
{
    // http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/
    Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
    Mat temp(img_in.size(), CV_8UC1);
    bool done = false;
    do
    {
        morphologyEx(img_in, temp, MORPH_OPEN, element);
        bitwise_not(temp, temp);
        bitwise_and(img_in, temp, temp);
        bitwise_or(img_out, temp, img_out);
        erode(img_in, img_in, element);

        double max;
        minMaxLoc(img_in, 0, &max);
        done = (max == 0);
    } while (!done);
    return;
}

void stairDetector::draw_lines(cv::Mat &image, const Lines &lines, const cv::Scalar &color)
{
    for (int i = 0; i < lines.size(); i++)
    {
        line(image, lines[i].p1, lines[i].p2, color, 2, 8);
    }
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
        // cout << lines[i];
    }
    ROS_INFO_ONCE("Found %d lines", lines.size());

    return;
}

void stairDetector::cluster_by_kmeans(const cv::Mat &img, Lines &lines, vector<Lines> &clustered_lines)
{
    int i, j;
    Mat points, labels, centers;
    const int MAX_CLUSTERS = 8;

    Scalar colorTab[] =
        {
            Scalar(100, 0, 255),
            // Scalar(255, 100, 0),
            Scalar(0, 0, 255),
            Scalar(100, 100, 0),
            Scalar(0, 255, 0),
            Scalar(255, 100, 100),
            Scalar(255, 0, 255),
            Scalar(0, 255, 255),
            Scalar(100, 100, 255)};

    // Mat bg_img(320, 240, CV_8UC3, Scalar(0, 0, 0));

    std::vector<cv::Point2f> mid_pts;

    // need to add slope parameter !
    for (i = 0; i < lines.size(); i++)
    {
        points.push_back(lines[i].p_mid);
    }

    double compactness = kmeans(points, MAX_CLUSTERS, labels,
                                TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 50, 1.0),
                                3, KMEANS_PP_CENTERS, centers);

    // imshow("mid points", img);
    // waitKey(30);
    // cout << points << endl;
    // cout << "-----------------" << endl;
    // cout << labels << endl;
    // cout << "-----------------" << endl;
    // cout << centers << endl;
    // cout << "-----------------" << endl;
    vector<int> ids;
    // std::vector<Lines> clustered_lines;
    Lines tmp;

    // img = Scalar::all(0);
    for (i = 0; i < lines.size(); i++)
    {
        int clusterIdx = labels.at<int>(i);
        lines[i].cluster_id = clusterIdx;
    }

    for (j = 0; j < MAX_CLUSTERS; j++)
    {
        tmp.clear();
        for (i = 0; i < lines.size(); i++)
        {
            if (lines[i].cluster_id == j)
                tmp.push_back(lines[i]);
        }
        clustered_lines.push_back(tmp);
    }

    for (j = 0; j < MAX_CLUSTERS; j++)
    {
        // ROS_INFO("%d", clustered_lines[j].size());
        if (clustered_lines[j].size() > 3)
        {

            for (i = 0; i < clustered_lines[j].size(); i++)
            {

                Point ipt = clustered_lines[j][i].p_mid;
                circle(img, ipt, 10, colorTab[j], 2, 8);
            }
        }
        else
        {
            ROS_INFO("Not enough points in cluster ");
        }
    }

    return;
}


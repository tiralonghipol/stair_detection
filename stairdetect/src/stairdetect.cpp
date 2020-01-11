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
    n.getParam("num_line_clusters", _max_clusters);

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

    RNG rng(0xFFFFFFFF);
    // vector<Scalar> colorTab;

    for (int i = 0; i < _max_clusters; i++)
    {
        _colorTab.push_back(random_color(rng));
    }
    // ROS_WARN("COLOR TAB SIZE = %d", _colorTab.size());
    // for (int i = 0; i < _colorTab.size(); i++)
    // {
    //     cout << _colorTab[i] << endl;
    // }

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
    ROS_INFO("In timing trigger callback");
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
    // cvtColor(edge_img, edge_img, CV_BINA);

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

        vector<Lines> processed_lines;
        process_clustered_lines(clustered_lines, processed_lines);

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

void stairDetector::process_clustered_lines(
    const vector<Lines> &clustered_lines,
    vector<Lines> &processed_lines)
{
    processed_lines.clear();
    for (int i = 0; i < clustered_lines.size(); i++)
    {

        // std::cout << "\nCLUSTER:\t" << i << std::endl;

        // remove lines in cluster which do not meet geometric constraints
        Lines filt_lines = filter_lines_by_angle(clustered_lines[i]);
        filt_lines = filter_lines_by_mid_pts_dist(filt_lines);
        processed_lines.push_back(filt_lines);
    }

    // get covariance matrix for each filtered cluster
    vector<Eigen::Matrix2d> cov_mats;
    for (int i = 0; i < processed_lines.size(); i++)
    {
        if (processed_lines[i].size() > 3)
        {
            Eigen::Matrix2d sigma = calc_covariance_matrix(processed_lines[i]);
            cov_mats.push_back(sigma);
            // std::cout << "\ncovariance for cluster: " << i << std::endl;
            // std::cout << sigma(0, 0) << ",\t" << sigma(0, 1) << std::endl;
            // std::cout << sigma(1, 0) << ",\t" << sigma(1, 1) << std::endl;

            // eigenvalues of covariance matrix
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(sigma);
            Eigen::Vector2d e_vals = eigensolver.eigenvalues();
            // std::cout << "eigenvalues for cluster: " << i << std::endl;
            // std::cout << e_vals << std::endl;
        }
    }

    return;
}

Lines stairDetector::filter_lines_by_mid_pts_dist(
    const Lines &lines)
{
    Mat distances(lines.size(), lines.size(), CV_32F), row;
    Lines filt_lines;
    double dx, dy, mag;
    vector<double> min_dists;

    if (lines.size() > 3)
    {
        for (int i = 0; i < lines.size(); i++)
        {
            for (int j = 0; j < lines.size(); j++)
            {
                if (j > i)
                {
                    dx = lines[i].p_mid.x - lines[j].p_mid.x;
                    dy = lines[i].p_mid.y - lines[j].p_mid.y;
                    mag = sqrt(dx * dx + dy * dy);
                    distances.at<float>(i, j) = mag;
                }
                else
                {
                    distances.at<float>(i, j) = 0;
                }
            }
        }
        // diagonal is all zeros
        for (int i = 0; i < distances.rows - 1; i++)
        {
            // initialize the minimum element as big
            int minm = 999;

            for (int j = i + 1; j < distances.cols; j++)
            {
                //  check for minimum  in each row of the matrix
                if (distances.at<float>(i, j) < minm && distances.at<float>(i, j) > 0)
                    minm = distances.at<float>(i, j);
            }
            if (minm > 0)
            {
                min_dists.push_back(minm);
            }
            else
            {
                min_dists.push_back(0);
            }
        }

        for (int i = 0; i < lines.size(); i++)
        {
            std::cout << "min dist = " << min_dists[i] << "\t";
            for (int j = 0; j < lines.size(); j++)
            {
                std::cout << distances.at<float>(i, j) << " ";
            }
            std::cout << "\n";
        }
        std::cout << "---------------------"
                  << "\n";
    }
    

    accumulator_set<double, stats<tag::variance>> acc;
    for_each(min_dists.begin(), min_dists.end(), bind<void>(ref(acc), _1));

    cout << boost::mean(acc) << endl;
    cout << sqrt(boost::variance(acc)) << endl;

    min_dists.clear();
    filt_lines = lines;
    return filt_lines;
}

Lines stairDetector::filter_lines_by_angle(
    const Lines &lines)
{
    // filtered lines have outliers (lines perpendicular to majority removed)
    Lines filt_lines;

    // element i, j is dot product between normals of lines i, j
    Eigen::MatrixXf normal_dot_prods(lines.size(), lines.size());
    for (int i = 0; i < lines.size(); i++)
    {
        normal_dot_prods(i, i) = 1.0;
        for (int j = i + 1; j < lines.size(); j++)
        {
            double dot_prod = 0;
            Vec2d n_i = get_normal_unit_vector(lines[i]);
            Vec2d n_j = get_normal_unit_vector(lines[j]);
            dot_prod = n_i[0] * n_j[0] + n_i[1] * n_j[1];
            normal_dot_prods(i, j) = abs(dot_prod);
            normal_dot_prods(j, i) = 0.0;
            // std::cout << "normal dot prod: \t" << dot_prod << std::endl;
        }
    }

    // element i, j is dot product between normal of i
    // and distance vector between i and j
    Eigen::MatrixXf normal_dist_dot_prods(lines.size(), lines.size());
    for (int i = 0; i < lines.size(); i++)
    {
        normal_dist_dot_prods(i, i) = 1.0;
        for (int j = i + 1; j < lines.size(); j++)
        {
            double dot_prod = 0;
            Vec2d n_i = get_normal_unit_vector(lines[j]);
            Vec2d d_ij = get_dist_unit_vector(lines[i], lines[j]);
            dot_prod = n_i[0] * d_ij[0] + n_i[1] * d_ij[1];
            normal_dist_dot_prods(i, j) = abs(dot_prod);
            normal_dist_dot_prods(j, i) = 0.0;
            // std::cout << "dot prod: \t" << dot_prod << std::endl;
        }
    }

    for (int i = 0; i < lines.size(); i++)
    {
        double norm_prod_sum = 0;
        double dist_prod_sum = 0;
        for (int j = 0; j < lines.size(); j++)
        {
            norm_prod_sum += normal_dot_prods(i, j);
            dist_prod_sum += normal_dist_dot_prods(i, j);
        }

        // don't add outliers w.r.t. normal dot prods, dist dot prods
        if ((norm_prod_sum - 1.0) / double(lines.size() - 1.0) > 0.3 && (dist_prod_sum - 1.0) / double(lines.size() - 1.0) > 0.0)
        {
            filt_lines.push_back(lines[i]);
        }
    }

    // std::cout << "initial number of lines:\t" << lines.size() << std::endl;
    // std::cout << "number of lines after filter:\t" << filt_lines.size() << std::endl;

    return filt_lines;
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
    dilate(
        img_out,
        img_out,
        Mat(_param.morph_kernel_size, _param.morph_kernel_size, CV_8UC1),
        Point(-1, -1),
        1,
        1,
        1);
    erode(
        img_in,
        img_out,
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
    Mat img;
    img_in.copyTo(img);

    cv::threshold(img, img, 0, 255, THRESH_BINARY);
    Mat temp;
    Mat eroded;

    Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool done = false;
    do
    {
        erode(img, eroded, element);
        dilate(eroded, temp, element);
        subtract(img, temp, temp);
        bitwise_or(img_out, temp, img_out);
        eroded.copyTo(img);

        done = (countNonZero(img) == 0);
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

Scalar stairDetector::random_color(RNG &rng)
{
    int icolor = (unsigned)rng;
    return Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);
}

void stairDetector::cluster_by_kmeans(
    const cv::Mat &img, Lines &lines,
    vector<Lines> &clustered_lines)
{
    int i, j;
    Mat labels, centers;

    // Method 1) use only x,y
    Mat points;
    for (i = 0; i < lines.size(); i++)
    {
        points.push_back(lines[i].p_mid);
    }

    // Method 2) use both x,y and slope
    // Mat points(lines.size(), 2, CV_32F);
    // for (i = 0; i < lines.size(); i++)
    // {
    // points.at<float>(i, 0) = lines[i].p_mid.x;
    // points.at<float>(i, 1) = lines[i].p_mid.y;
    // points.at<float>(i, 2) = lines[i].k;
    // }

    if (points.rows >= _max_clusters)
    {
        double compactness = kmeans(points, _max_clusters, labels,
                                    TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 50, 1.0),
                                    3, KMEANS_PP_CENTERS, centers);
        Lines tmp;

        for (i = 0; i < lines.size(); i++)
        {
            int clusterIdx = labels.at<int>(i);
            lines[i].cluster_id = clusterIdx;
        }

        for (j = 0; j < _max_clusters; j++)
        {
            tmp.clear();
            for (i = 0; i < lines.size(); i++)
            {
                if (lines[i].cluster_id == j)
                    tmp.push_back(lines[i]);
            }
            clustered_lines.push_back(tmp);
        }

        vector<Lines> processed_lines;
        process_clustered_lines(clustered_lines, processed_lines);

        for (j = 0; j < _max_clusters; j++)
        {
            // ROS_INFO("%d", clustered_lines[j].size());
            if (processed_lines[j].size() > 3)
            {
                for (i = 0; i < processed_lines[j].size(); i++)
                {
                    // Point ipt = clustered_lines[j][i].p_mid;
                    Point ipt = processed_lines[j][i].p_mid;
                    circle(img, ipt, 10, _colorTab[j], 2, 8);
                }
            }
        }
    }
    else
    {
        ROS_INFO("Not enough points to perform kmeans ");
    }
    return;
}

Eigen::Matrix2d stairDetector::calc_covariance_matrix(const Lines &lines)
{
    Eigen::Matrix2d cov_mat;
    double x_bar;
    double y_bar;

    // calculate x, y, means
    for (int i = 0; i < lines.size(); i++)
    {
        x_bar += lines[i].p_mid.x;
        y_bar += lines[i].p_mid.y;
    }
    x_bar /= float(lines.size());
    y_bar /= float(lines.size());

    // build data matrix
    Eigen::MatrixXd data_mat(lines.size(), 2);
    for (int i = 0; i < lines.size(); i++)
    {
        data_mat(i, 0) = lines[i].p_mid.x - x_bar;
        data_mat(i, 1) = lines[i].p_mid.y - y_bar;
    }

    cov_mat = data_mat.transpose() * data_mat;
    cov_mat /= float(lines.size());
    return cov_mat;
}

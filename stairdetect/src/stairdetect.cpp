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
    n.getParam("topic_staircase_polygon", _topic_polygon);

    // frames
    n.getParam("frame_world", _frame_world);
    n.getParam("frame_pcl", _frame_pcl);

    // other params
    n.getParam("stair_detect_timing", _stair_detect_time);
    n.getParam("num_line_clusters", _max_clusters);
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
    // _pub_staircase_polygon = n.advertise<geometry_msgs::PolygonStamped>(_topic_polygon, bufSize);
    _pub_staircase_polygon = n.advertise<geometry_msgs::PolygonStamped>(_topic_polygon, 10);

    image_transport::ImageTransport it_0(n);
    _pub_bird_view_img = it_0.advertise(_topic_bird_eye_img, 1);
    image_transport::ImageTransport it_1(n);
    _pub_proc_img = it_1.advertise(_topic_proc_img, 1);
    image_transport::ImageTransport it_2(n);
    _pub_line_img = it_2.advertise(_topic_line_img, 1);
    // show final result of filtering on this topic
    image_transport::ImageTransport it_3(n);
    _pub_filtered_line_img = it_3.advertise(_topic_filtered_line_img, 1);

    // timers
    _timer_stair_detect = n.createTimer(
        ros::Duration(_stair_detect_time), &stairDetector::callback_timer_trigger, this);

    RNG rng(0xFFFFFFFF);
    // vector<Scalar> colorTab;

    for (int i = 0; i < _max_clusters; i++)
    {
        _colorTab.push_back(random_color(rng));
    }
    setParam(_param);
}

void stairDetector::callback_stitched_pcl(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
{
    if (!msg->empty())
    {
        // std::cout << msg->header.frame_id << std::endl;
        _recent_pose = _pose_Q[0];
        try
        {
            _tf_listener.waitForTransform(_frame_world, _frame_pcl, ros::Time(0), ros::Duration(0,2));
            _tf_listener.lookupTransform(_frame_world, _frame_pcl, ros::Time(0), _recent_tf);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("Error finding transform %s", ex.what());
        }
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
    Mat img(
        int(_param.img_xy_dim / _param.img_resolution),
        int(_param.img_xy_dim / _param.img_resolution),
        CV_8UC1,
        Scalar(0));

    if (!_recent_cloud->empty())
    {
        _callback_tf = _recent_tf;
        _callback_pose = _recent_pose;

        // convert most recently-received pointcloud to birds-eye-view image
        pcl_to_bird_view_img(_recent_cloud, img);

        // find convex hulls around line clusters
        vector<vector<vector<cv::Point>>> hulls;
        hulls = filter_img(img);

        // find areas, centroids of potential staircases (in pixels)
        vector<double> hull_areas_px;
        vector<cv::Point> hull_centroids_px;
        for (int i = 0; i < hulls.size(); i++)
        {
            if (hulls[i][0].size() > 2)
            {
                hull_centroids_px.push_back(calc_centroid_pixel(hulls[i][0]));
                hull_areas_px.push_back(contourArea(hulls[i][0], false));
            }
        }

        px_to_m_and_publish(hulls, hull_centroids_px);

        // check recent potential staircases against previously-found staircases
        // TODO
    }
    else
    {
        ROS_INFO("No stitch in progress");
    }
    return;
}

void stairDetector::px_to_m_and_publish(
    vector<vector<vector<cv::Point>>> hulls,
    vector<cv::Point> hull_centroids_px)
{
    vector<vector<tf::Vector3>> hulls_rf(hulls.size());
    vector<vector<tf::Vector3>> hulls_wf(hulls.size());

    Eigen::Vector2d xy;
    for( int i = 0; i < hulls.size(); i++ )
    {
        for( int j = 0; j < hulls[i][0].size(); j++ )
        {
            xy = px_to_m(hulls[i][0][j]);
            hulls_rf[i].push_back(tf::Vector3(xy[0], xy[1], _callback_pose.pose.pose.position.z));
        }

        // for( int j = 0; j < hulls_rf[i].size(); j++ )
        // {
            // hulls_wf[i].push_back(_callback_tf * hulls_rf[i][j]);
            // hulls_wf[i].push_back(_callback_tf.inverse() * hulls_rf[i][j]);
        // }
    }
    std::cout << "\nDONE BUILDING HULLS IN WORLD FRAME" << std::endl;
    // std::cout << hulls_wf.size() << std::endl;

    // publish the hulls as rviz polygons
    int j = 0;
    while( j < hulls_rf.size() )
    {
        if( hulls_rf[j].size() > 0 )
        {
            std::cout << j << std::endl;
            geometry_msgs::PolygonStamped msg;
            msg = hull_to_polygon_msg(hulls_rf[j]);
            // msg = hull_to_polygon_msg(hulls_wf[j]);
            _pub_staircase_polygon.publish(msg);
            j++;
        }
    }

    return;
}

geometry_msgs::PolygonStamped stairDetector::hull_to_polygon_msg( 
    const vector<tf::Vector3> & hull )
{
    geometry_msgs::PolygonStamped msg;
    msg.header.stamp = ros::Time(0);
    msg.header.frame_id = _frame_world;

    for( int i = 0; i < hull.size(); i++ )
    {
        geometry_msgs::Point32 pt;
        pt.x = hull[i][0];
        pt.y = hull[i][1];
        pt.z = hull[i][2];
        msg.polygon.points.push_back(pt);
    }

    return msg;
}

Eigen::Vector2d stairDetector::px_to_m(
    const cv::Point & pt)
{
    Eigen::Vector2d xy;
    int img_midpt = int(_param.img_xy_dim / _param.img_resolution) / 2;
    xy[0] = _callback_pose.pose.pose.position.x
        - (float(pt.y - img_midpt)) * _param.img_resolution;
    xy[1] = + _callback_pose.pose.pose.position.y
        - (float(pt.x - img_midpt)) * _param.img_resolution;
    return xy;
}

void stairDetector::pcl_to_bird_view_img(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    cv::Mat &img)
{
    int img_midpt = int(_param.img_xy_dim / _param.img_resolution) / 2;

    // get current xy
    double x_0 = _callback_pose.pose.pose.position.x;
    double y_0 = _callback_pose.pose.pose.position.y;

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

    _param.max_stair_width = config.max_stair_width;
    setParam(_param);
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

vector<vector<vector<cv::Point>>> stairDetector::filter_img(cv::Mat &raw_img)
{
    // calculates convex hulls for potential staircases
    vector<vector<vector<cv::Point>>> hulls;

    // Mat proc_img, line_img, filtered_line_img;
    Mat line_img, filtered_line_img;

    Mat proc_img = raw_img.clone();
    // Mat filtered_line_img = raw_img.clone();
    // edge detection
    // canny_edge_detect(img, edge_img);

    // Mat morph_img(raw_img.size(), CV_8UC1);
    // morph_filter(raw_img, morph_img);
    // Mat t_img;
    // threshold(morph_img, t_img, 0, 255, THRESH_BINARY);
    // skeleton_filter(morph_img, proc_img);
    // ---------------------------------------- //
    morph_filter(raw_img, proc_img);

    // medianBlur(edge_img, edge_img, 3);
    // cvtColor(edge_img, edge_img, CV_BINA);

    // line detection
    Lines lines;
    lsd_lines(proc_img, lines);

    if (lines.size() > 3)
    {
        // plot lines on image
        cvtColor(proc_img, line_img, CV_GRAY2BGR);
        cvtColor(proc_img, filtered_line_img, CV_GRAY2BGR);
        draw_lines(line_img, lines, Scalar(255, 0, 0));

        // line clustering
        vector<Lines> clustered_lines;
        cluster_by_kmeans(line_img, lines, clustered_lines);
        draw_clustered_lines(line_img, clustered_lines);

        // sub-clustering based on line orientation
        vector<Lines> subclustered_lines;
        subclustered_lines = subcluster_by_orientation(clustered_lines);

        // additional filtering on lines within clusters
        vector<Lines> processed_clusters;
        process_clustered_lines(subclustered_lines, processed_clusters);

        // draw processed clusters on image
        for (int i = 0; i < processed_clusters.size(); i++)
        {
            draw_lines(filtered_line_img, processed_clusters[i], Scalar(0, 0, 255));
        }

        // find convex hulls around line clusters
        for (int i = 0; i < processed_clusters.size(); i++)
        {
            hulls.push_back(calc_cluster_bounds(processed_clusters[i]));
        }
        draw_clustered_lines(filtered_line_img, processed_clusters);

        // draw convex hulls on the image
        for (int i = 0; i < processed_clusters.size(); i++)
        {
            if (processed_clusters[i].size() > 0)
            {
                drawContours(filtered_line_img, hulls[i], -1, Scalar(0, 255, 0), 2);
            }
        }

        // publish images
        if (_param.debug)
        {
            publish_img_msgs(raw_img, proc_img, line_img, filtered_line_img);
        }
    }
    else
    {
        ROS_WARN("Not enough lines detected");
    }

    return hulls;
}

cv::Point stairDetector::calc_centroid_pixel(
    const vector<cv::Point> &hull)
{
    // calculate the centroid of the convex hull provided
    cv::Moments m = cv::moments(hull, true);
    cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
    return center;
}

vector<vector<cv::Point>> stairDetector::calc_cluster_bounds(
    const Lines &lines)
{
    // some data format conversion
    vector<vector<cv::Point>> line_endpoints;
    vector<cv::Point> pts;
    for (int i = 0; i < lines.size(); i++)
    {
        double x_1 = lines[i].p1.x;
        double x_2 = lines[i].p2.x;
        double y_1 = lines[i].p1.y;
        double y_2 = lines[i].p2.y;
        cv::Point p1(x_1, y_1);
        cv::Point p2(x_2, y_2);
        vector<cv::Point> temp(2);
        temp[0] = p1;
        temp[1] = p2;
        pts.push_back(p1);
        pts.push_back(p2);
    }
    line_endpoints.push_back(pts);

    // compute the convex hull
    vector<vector<cv::Point>> hull(line_endpoints.size());
    for (int i = 0; i < line_endpoints.size(); i++)
    {
        if (line_endpoints[i].size() > 0)
        {
            convexHull(Mat(line_endpoints[i]), hull[i], false);
        }
    }

    return hull;
}

void stairDetector::process_clustered_lines(
    const vector<Lines> &clustered_lines,
    vector<Lines> &processed_lines)
{
    processed_lines.clear();

    for (int i = 0; i < clustered_lines.size(); i++)
    {
        // remove lines in each cluster which do not meet geometric constraints
        Lines filt_lines = clustered_lines[i];
        // Lines filt_lines;
        // filt_lines = filter_lines_by_mid_pts_dist(clustered_lines[i]);
        // filt_lines = filter_lines_by_max_width(filt_lines);
        filt_lines = filter_lines_by_gransac(clustered_lines[i]);
        filt_lines = filter_lines_by_mid_pts_dist(filt_lines);
        processed_lines.push_back(filt_lines);
    }

    return;
}

Lines stairDetector::filter_lines_by_covariance(
    const Lines &lines)
{
    Lines filt_lines;

    // get covariance matrix for each cluster
    if (lines.size() > 3)
    {
        Eigen::Matrix2d sigma = calc_covariance_matrix(lines);
        // cov_mats.push_back(sigma);
        // std::cout << "\ncovariance for cluster: " << std::endl;
        // std::cout << sigma(0, 0) << ",\t" << sigma(0, 1) << std::endl;
        // std::cout << sigma(1, 0) << ",\t" << sigma(1, 1) << std::endl;

        // eigenvalues of covariance matrix
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(sigma);
        Eigen::Vector2d e_vals = eigensolver.eigenvalues();
        // std::cout << "eigenvalues for cluster: " << std::endl;
        // std::cout << e_vals << std::endl;
    }
    return filt_lines;
}

Lines stairDetector::filter_lines_by_max_width(
    const Lines &lines)
{
    Lines filt_lines = lines;

    for (int i = 0; i < lines.size(); i++)
    {
        ROS_WARN("filt_lines[%d].lenght = %f", i, lines[i].length);
        if (filt_lines[i].length * _param.img_resolution > _param.max_stair_width)
            filt_lines.erase(filt_lines.begin() + i);
    }

    return filt_lines;
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
        for (int i = 0; i < lines.size() - 1; i++)
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

            for (int j = 0; j < distances.cols; j++)
            {
                // if (i == j)
                //     continue;
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

        // for (int i = 0; i < lines.size() - 1; i++)
        // {
        //     std::cout << "min dist = " << min_dists[i] << "\t";
        //     for (int j = 0; j < lines.size(); j++)
        //     {
        //         std::cout << distances.at<float>(i, j) << " ";
        //     }
        //     std::cout << "\n";
        // }

        // std::cout << "---------------------------------"
        //           << "\n";
        // double sum = 0;
        // double mean = 0;
        double sum = std::accumulate(min_dists.begin(), min_dists.end(), 0.0);
        double mean = sum / min_dists.size();

        std::vector<double> diff(min_dists.size());
        std::transform(min_dists.begin(), min_dists.end(), diff.begin(), [mean](double x) { return x - mean; });

        double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / min_dists.size());
        cout << "Mean = " << mean << endl;
        cout << "Variance = " << stdev << endl;
        std::cout << "---------------------------------"
                  << "\n";

        if (stdev < 20)
        {
            // std::cout << "Potential Stair !" << endl;
            filt_lines = lines;
        }
        else
        {
            filt_lines = {};
        }
    }

    min_dists.clear();
    return filt_lines;
}

vector<Lines> stairDetector::subcluster_by_orientation(
    const vector<Lines> &clustered_lines)
{
    vector<Lines> new_clusters;
    for (int i = 0; i < clustered_lines.size(); i++)
    {
        vector<Lines> temp;
        temp = filter_lines_by_angle(clustered_lines[i]);
        if (temp[0].size() > 0)
        {
            new_clusters.push_back(temp[0]);
        }
        if (temp[1].size() > 0)
        {
            new_clusters.push_back(temp[1]);
        }
    }
    return new_clusters;
}

vector<Lines> stairDetector::filter_lines_by_angle(
    const Lines &lines)
{
    // filtered lines have outliers (lines perpendicular to majority removed)
    Lines filt_lines_0;
    Lines filt_lines_1;
    vector<Lines> angular_clusters;

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
        }
    }

    double max_sum = 0;
    int max_row = 0;

    for (int i = 0; i < lines.size(); i++)
    {
        double norm_prod_sum = 0;
        for (int j = 0; j < lines.size(); j++)
        {
            norm_prod_sum += normal_dot_prods(i, j);
        }

        if (norm_prod_sum > max_sum)
        {
            max_row = i;
            max_sum = norm_prod_sum;
        }
    }

    // separate lines in cluster by orientation
    for (int i = 0; i < lines.size(); i++)
    {
        if (normal_dot_prods(max_row, i) > 0.7071)
        {
            filt_lines_0.push_back(lines[i]);
        }
        else
        {
            filt_lines_1.push_back(lines[i]);
        }
    }

    angular_clusters.push_back(filt_lines_0);
    angular_clusters.push_back(filt_lines_1);

    return angular_clusters;

    // // element i, j is dot product between normal of i
    // // and distance vector between i and j
    // Eigen::MatrixXf normal_dist_dot_prods(lines.size(), lines.size());
    // for (int i = 0; i < lines.size(); i++)
    // {
    //     normal_dist_dot_prods(i, i) = 1.0;
    //     for (int j = i + 1; j < lines.size(); j++)
    //     {
    //         double dot_prod = 0;
    //         Vec2d n_i = get_normal_unit_vector(lines[j]);
    //         Vec2d d_ij = get_dist_unit_vector(lines[i], lines[j]);
    //         dot_prod = n_i[0] * d_ij[0] + n_i[1] * d_ij[1];
    //         normal_dist_dot_prods(i, j) = abs(dot_prod);
    //         normal_dist_dot_prods(j, i) = 0.0;
    //         // std::cout << "dot prod: \t" << dot_prod << std::endl;
    //     }
    // }
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
        img_in,
        img_out,
        Mat(_param.morph_kernel_size, _param.morph_kernel_size, CV_8UC1),
        Point(-1, -1),
        1,
        1,
        1);
    erode(
        img_out,
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

void stairDetector::draw_clustered_lines(cv::Mat &image, const vector<Lines> &clustered_lines)
{
    int i, j;
    for (j = 0; j < clustered_lines.size(); j++)
    {
        for (i = 0; i < clustered_lines[j].size(); i++)
        {
            Point ipt = clustered_lines[j][i].p_mid;
            circle(image, ipt, 8, _colorTab[j], 2, 8);
        }
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
    Mat slopes(lines.size(), 3, CV_32F);

    for (i = 0; i < lines.size(); i++)
    {
        points.push_back(lines[i].p_mid);
        // slopes.push_back(lines[i].k);
    }
    // Mat slopes;

    // Method 2) use both x,y and slope
    for (i = 0; i < lines.size(); i++)
    {
        slopes.at<float>(i, 0) = lines[i].p_mid.x;
        slopes.at<float>(i, 1) = lines[i].p_mid.y;
        if (isinf(lines[i].k))
            slopes.at<float>(i, 2) = 9999;
        else
            slopes.at<float>(i, 2) = lines[i].k;
    }

    // std::cout << slopes << std::endl;

    if (points.rows >= _max_clusters)
    {
        double compactness = kmeans(points, _max_clusters, labels,
                                    TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 10.0),
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

bool stairDetector::get_bounding_box(const cv::Mat &input_image, const Lines &lines, std::vector<cv::Point> &bounding_box)

{
    bounding_box.clear();
    std::vector<int> valid_ids;

    int left_most = input_image.cols;
    int right_most = -1;
    int up_most = input_image.rows;
    int down_most = -1;
    vector<double> x_vals;

    for (int i = 0; i < lines.size(); i++)
    {
        for (int j = 0; j < lines[i].pixels.size(); j++)
        {

            int x1 = lines[i].p1.x;
            int x2 = lines[i].p2.x;
            if (x1 < left_most)
            {
                left_most = x1;
            }
            if (x1 > right_most)
            {
                right_most = x1;
            }
            if (x2 < left_most)
            {
                left_most = x2;
            }
            if (x2 > right_most)
            {
                right_most = x2;
            }
            valid_ids.push_back(i);
            break;
        }
    }
    for (int i = 0; i < valid_ids.size(); i++)
    {
        int id = valid_ids[i];
        // ignore lines if it's length is smaller than 20
        if (lines[id].length < 20)
        {
            continue;
        }
        for (int j = 0; j < lines[id].pixels.size(); j++)
        {
            int x = lines[id].pixels[j].x;
            int y = lines[id].pixels[j].y;
            if (x < left_most || x > right_most)
            {
                break;
            }
            if (y < up_most)
            {
                up_most = y;
            }
            if (y > down_most)
            {
                down_most = y;
            }
        }
    }
    cv::Point p1(left_most, up_most);
    cv::Point p2(right_most, down_most);

    bounding_box.push_back(p1);
    bounding_box.push_back(p2);
    return true;
}

void stairDetector::draw_bounding_box(cv::Mat &image, const std::vector<cv::Point> &bounding_box)
{
    if (bounding_box.size() != 2)
    {
        if (_param.debug)
        {
            std::cout << "Can't draw boxes because the point size is not equal to 2" << std::endl;
        }
        return;
    }
    cv::rectangle(image, bounding_box[0], bounding_box[1], cv::Scalar(0, 255, 0), 3, 8);
}

Lines stairDetector::filter_lines_by_gransac(const Lines &lines_in)
{
    vector<Point> mid_points;
    Vec4f inliers;
    Lines lines_out;

    cv::Mat Canvas(500, 500, CV_8UC3);
    Canvas.setTo(255);

    for (auto r : lines_in)
        mid_points.push_back(r.p_mid);

    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
    for (int i = 0; i < lines_in.size(); ++i)
    {
        Point Pt(lines_in[i].p_mid.x, lines_in[i].p_mid.y);
        circle(Canvas, Pt, 8, Scalar(0, 255, 0), 2, 8);

        std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point2D>(Pt.x, Pt.y);
        CandPoints.push_back(CandPt);
    }

    GRANSAC::RANSAC<Line2DModel, 2> Estimator;
    Estimator.Initialize(8, 100); // Threshold, iterations
    int64_t start = cv::getTickCount();
    Estimator.Estimate(CandPoints);
    int64_t end = cv::getTickCount();
    std::cout << "RANSAC took: " << GRANSAC::VPFloat(end - start) / GRANSAC::VPFloat(cv::getTickFrequency()) * 1000.0 << " ms." << std::endl;

    auto BestInliers = Estimator.GetBestInliers();
    if (BestInliers.size() > 0)
    {
        for (auto &Inlier : BestInliers)
        {
            auto RPt = std::dynamic_pointer_cast<Point2D>(Inlier);
            cv::Point2f Pt(RPt->m_Point2D[0], RPt->m_Point2D[1]);
        }
        lines_out.clear();
        for (auto &Inlier : BestInliers)
        {
            auto RPt = std::dynamic_pointer_cast<Point2D>(Inlier);
            // cout << "x =" << RPt->m_Point2D[0]
            //  << "y =" << RPt->m_Point2D[1] << endl;
            Point2f Pt(RPt->m_Point2D[0], RPt->m_Point2D[1]);
            for (auto line : lines_in)
                if (line.p_mid == Pt)
                {
                    lines_out.push_back(line);
                }
        }
    }
    else
    {
        ROS_WARN("Not enough inliers running ransac");
    }

    cout << "Size Lines IN = " << lines_in.size() << endl;
    cout << "Size Lines OUT = " << lines_out.size() << endl;

    // lines_out = lines_in;
    return lines_out;
}

void stairDetector::DrawFullLine(cv::Mat &img, cv::Point a, cv::Point b, cv::Scalar color, int LineWidth)
{
    GRANSAC::VPFloat slope = Slope(a.x, a.y, b.x, b.y);

    cv::Point p(0, 0), q(img.cols, img.rows);

    p.y = -(a.x - p.x) * slope + a.y;
    q.y = -(b.x - q.x) * slope + b.y;

    cv::line(img, p, q, color, LineWidth, cv::LINE_AA, 0);
}

GRANSAC::VPFloat stairDetector::Slope(int x0, int y0, int x1, int y1)
{
    return (GRANSAC::VPFloat)(y1 - y0) / (x1 - x0);
}
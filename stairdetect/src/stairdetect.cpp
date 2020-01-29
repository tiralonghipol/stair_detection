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
    n.getParam("topic_staircase_centroid", _topic_centroid_vis);
    n.getParam("topic_staircase_centroid_pose", _topic_centroid_pose);

    // frames
    n.getParam("frame_world", _frame_world);
    n.getParam("frame_pcl", _frame_pcl);

    // other params
    n.getParam("stair_detect_timing", _stair_detect_time);
    n.getParam("max_clusters", _max_clusters);
    n.getParam("min_stair_steps", _min_stair_steps);
    _pose_Q_size = 40;
    _confirmed_centroids.clear();
    _unconfirmed_centroids.clear();

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
        if (_param.debug)
            ROS_INFO("Stitched PCL not received yet");
        sharedPtr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(_topic_stitched_pcl, ros::Duration(2));
    }

    // dynamic reconfigure
    _dyn_rec_cb = boost::bind(&stairDetector::callback_dyn_reconf, this, _1, _2);
    _dr_srv.setCallback(_dyn_rec_cb);

    // publishers
    _pub_trimmed_pcl = n.advertise<sensor_msgs::PointCloud2>(_topic_trimmed_pcl, bufSize);
    _pub_staircase_polygon = n.advertise<geometry_msgs::PolygonStamped>(_topic_polygon, 10);
    _pub_centroid_marker = n.advertise<visualization_msgs::Marker>(_topic_centroid_vis, 10);
    _pub_centroid_pose = n.advertise<geometry_msgs::PoseStamped>(_topic_centroid_pose, 10);

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

    for (int i = 0; i < _max_clusters; i++)
    {
        _color_tab.push_back(random_color(rng));
    }
    set_param(_param);
}

void stairDetector::callback_stitched_pcl(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // outrem.setInputCloud(msg);
    // outrem.setRadiusSearch(10.0);
    // outrem.setMinNeighborsInRadius(500);
    // outrem.filter(*cloud_filtered);

    if (!msg->empty())
    {
        _recent_pose = _pose_Q[0];
        _recent_cloud = msg;
        // _recent_cloud = cloud_filtered;
    }
    else
    {
        if (_param.debug)
            ROS_INFO("Received pose message is empty");
    }
    return;
}

void stairDetector::callback_timer_trigger(
    const ros::TimerEvent &event)
{
    if (_recent_cloud->empty())
    {
        return;
    }

    Mat img(
        int(_param.img_xy_dim / _param.img_resolution),
        int(_param.img_xy_dim / _param.img_resolution),
        CV_8UC1,
        Scalar(0));

    _callback_pose = _recent_pose;

    // convert most recently-received pointcloud to birds-eye-view image
    pcl_to_bird_view_img(_recent_cloud, img);

    // find convex hulls around line clusters
    vector<vector<vector<cv::Point>>> hulls;
    hulls = filter_img(img);

    // find centroids of potential staircases (in pixels)
    vector<cv::Point> hull_centroids_px;
    for (int i = 0; i < hulls.size(); i++)
    {
        hull_centroids_px.push_back(calc_centroid_pixel(hulls[i][0]));
    }

    // convert bounds, centroids from pixels to world-frame coordinates
    vector<vector<Eigen::Vector3d>> hull_bounds_wf = hull_px_to_wf(hulls);
    vector<Eigen::Vector2d> hull_centroids_wf = hull_centroid_px_to_wf(hull_centroids_px);

    // publish convex hull polygons
    for (auto h : hull_bounds_wf)
    {   
        geometry_msgs::PolygonStamped hull_msg = hull_to_polygon_msg(h, 1);
        _pub_staircase_polygon.publish(hull_msg);
    }

    // publish any new centroids found
    process_and_publish_centroids(hull_centroids_wf);
    return;
}

void stairDetector::process_and_publish_centroids(
    const vector<Eigen::Vector2d> & hull_centroids_wf)
{
    // check for previously-found centroids
    if (_unconfirmed_centroids.empty()) 
    {
        for (auto h : hull_centroids_wf) 
        {
            _unconfirmed_centroids.push_back(h);
        }
    }
    else 
    {
        // check if centroid was already defined
        for (auto c_new : hull_centroids_wf) 
        {
            bool centroid_already_confirmed = false;
            for (auto c_conf : _confirmed_centroids)
            {
                double dx = c_new[0] - c_conf[0];
                double dy = c_new[1] - c_conf[1];
                if (sqrt(dx * dx + dy * dy) < _min_dist_between_stairs)
                {
                    centroid_already_confirmed = true;
                    // update confirmed centroid position as average of new and previous value
                    c_conf[0] = (c_conf[0] + c_new[0]) / 2;
                    c_conf[1] = (c_conf[1] + c_new[1]) / 2;
                }
            }          

            // if the centroid hasn't been found, check if it corresponds to an unconfirmed centroid
            if (! centroid_already_confirmed) 
            {
                vector<Eigen::Vector2d> new_unconf_centroids;
                for (int i = 0; i < _unconfirmed_centroids.size(); i++)
                {
                    double dx = c_new[0] - _unconfirmed_centroids[i][0];
                    double dy = c_new[1] - _unconfirmed_centroids[i][1];
                    // if the new centroid is close to an unconfirmed centroid,
                    // remove it from the unconfirmed list, publish it,
                    // and place it in the confirmed list
                    if (sqrt(dx * dx + dy * dy) < _min_dist_between_stairs)
                    {
                        // centroid has position = average of previous and new
                        Eigen::Vector2d centroid_to_publish;
                        centroid_to_publish[0] = (c_new[0] + _unconfirmed_centroids[i][0]) / 2.0;
                        centroid_to_publish[1] = (c_new[1] + _unconfirmed_centroids[i][1]) / 2.0;

                        // convert to message, publish
                        // visualization
                        visualization_msgs::Marker centroid_vis_msg = centroid_to_marker_msg(centroid_to_publish);
                        _pub_centroid_marker.publish(centroid_vis_msg);
                        // pose message for planner
                        geometry_msgs::PoseStamped centroid_pose_msg = hull_centroid_to_pose_msg(centroid_to_publish);
                        _pub_centroid_pose.publish(centroid_pose_msg);

                        // add newly confirmed centroid
                        _confirmed_centroids.push_back(centroid_to_publish);

                        // remove unconfirmed centroid
                        _unconfirmed_centroids.erase(_unconfirmed_centroids.begin() + i);
                    }
                    else
                    {
                        // otherwise, hold this newly unconfirmed centroid to be added to the list
                        new_unconf_centroids.push_back(c_new);
                    }
                }

                // add all new unconfirmed centroids
                for (auto c_new_unconf : new_unconf_centroids)
                {
                    _unconfirmed_centroids.push_back(c_new_unconf);
                }
            }  
        }
    }
    return;
}

// check whether hull area exceeds threshold
bool stairDetector::check_hull_area(
    const double &hull_area)
{
    // convert area in pixels^2 to m^2
    double area = hull_area * _param.img_resolution * _param.img_resolution;
    return area < _param.staircase_max_area && area > _param.staircase_min_area;
}

// convert a convex hull from pixel coordinates to world frame coordinates
vector<vector<Eigen::Vector3d>> stairDetector::hull_px_to_wf(
    const vector<vector<vector<cv::Point>>> & hulls)
{
    vector<vector<Eigen::Vector3d>> hulls_wf;
    Eigen::Vector2d xy;
    for (int i = 0; i < hulls.size(); i++)
    {
        vector<Eigen::Vector3d> hull;
        for (int j = 0; j < hulls[i][0].size(); j++ ) 
        {
            xy = px_to_m(hulls[i][0][j]);
            hull.push_back(Eigen::Vector3d(xy[0], xy[1], _callback_pose.pose.pose.position.z));
        }
        hulls_wf.push_back(hull);
    }
    return hulls_wf;
}

// convert a centroid of a convext hull from pixel coordinates to world frame coordinates
vector<Eigen::Vector2d> stairDetector::hull_centroid_px_to_wf(
    const vector<cv::Point> centroids_px)
{
    vector<Eigen::Vector2d> centroids_wf;
    for (int i = 0; i < centroids_px.size(); i++) 
    {
        centroids_wf.push_back(px_to_m(centroids_px[i]));
    }
    return centroids_wf;
}

// convert hull centroid to pose message for planner
geometry_msgs::PoseStamped stairDetector::hull_centroid_to_pose_msg(
    const Eigen::Vector2d & centroid)
{
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = _frame_world;
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = centroid[0];
    msg.pose.position.y = centroid[1];
    msg.pose.position.z = _callback_pose.pose.pose.position.z;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;
    return msg;
}

// convert a centroid in world frame to marker message
visualization_msgs::Marker stairDetector::centroid_to_marker_msg(
    const Eigen::Vector2d & centroid )
{
    visualization_msgs::Marker msg;
    msg.pose.position.x = centroid[0];
    msg.pose.position.y = centroid[1];
    msg.pose.position.z = _callback_pose.pose.pose.position.z;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;

    msg.header.frame_id = _frame_world;
    msg.header.stamp = ros::Time::now();
    msg.type = visualization_msgs::Marker::SPHERE;
    msg.lifetime = ros::Duration(5);

    msg.scale.x = 0.5;
    msg.scale.y = 0.5;
    msg.scale.z = 0.5;

    msg.color.a = 1.0;
    msg.color.g = 0.05;
    msg.color.b = 0.05;
    msg.color.r = 1.0;
    return msg;
}

// convert hull to polygon message
geometry_msgs::PolygonStamped stairDetector::hull_to_polygon_msg(
    const vector<Eigen::Vector3d> &hull,
    int seq_id)
{
    geometry_msgs::PolygonStamped msg;
    msg.header.stamp = ros::Time(0);
    msg.header.frame_id = _frame_world;
    msg.header.seq = seq_id;

    for (int i = 0; i < hull.size(); i++)
    {
        geometry_msgs::Point32 pt;
        pt.x = hull[i][0];
        pt.y = hull[i][1];
        pt.z = hull[i][2];
        msg.polygon.points.push_back(pt);
    }
    return msg;
}

// convert from pixels to world-frame coordinates
Eigen::Vector2d stairDetector::px_to_m(
    const cv::Point &pt)
{
    Eigen::Vector2d xy;
    int img_midpt = int(_param.img_xy_dim / _param.img_resolution) / 2;
    xy[0] = _callback_pose.pose.pose.position.x - (float(pt.y - img_midpt)) * _param.img_resolution;
    xy[1] = +_callback_pose.pose.pose.position.y - (float(pt.x - img_midpt)) * _param.img_resolution;
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
    if (_param.debug)
        ROS_INFO("Dynamic reconfigure triggered");
    // high level bools
    _param.debug = config.debug;

    // morphological
    _param.morph_kernel_size = config.morph_kernel_size;
    _param.morph_num_iter = config.morph_num_iter;

    //  median blur
    _param.median_kernel_size = config.median_kernel_size;

    // lsd
    _param.lsd_scale = config.lsd_scale;
    _param.lsd_sigma_scale = config.lsd_sigma_scale;
    _param.lsd_quant = config.lsd_quant;
    _param.lsd_angle_th = config.lsd_angle_th;
    _param.lsd_log_eps = config.lsd_log_eps;
    _param.lsd_density_th = config.lsd_density_th;

    // area constraints
    _param.staircase_max_area = config.staircase_max_area;
    _param.staircase_min_area = config.staircase_min_area;

    set_param(_param);
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

void stairDetector::set_param(const stairDetectorParams &param)
{
    _param = param;
    return;
}

// detect potential staircases and find convex hulls around them
vector<vector<vector<cv::Point>>> stairDetector::filter_img(cv::Mat &raw_img)
{
    vector<vector<vector<cv::Point>>> hulls;

    Mat line_img, filtered_line_img;
    Mat proc_img = raw_img.clone();

    Mat morph_img(raw_img.size(), CV_8UC1);
    medianBlur(raw_img, proc_img, _param.median_kernel_size);
    morph_filter(proc_img, proc_img);
    // cv::ximgproc::thinning(proc_img, proc_img);
    // imshow("Original", raw_img);   // Show our image inside it.
    // imshow("Processed", proc_img); // Show our image inside it.

    // line detection
    Lines lines;
    lsd_lines(proc_img, lines);

    if (lines.size() > _min_stair_steps)
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
        draw_clustered_lines(filtered_line_img, processed_clusters);

        // find convex hulls around line clusters
        for (int i = 0; i < processed_clusters.size(); i++)
        {
            vector<vector<cv::Point>> hull = calc_cluster_bounds(processed_clusters[i]);

            if (hull[0].size() > 2)
            {
                // only keep hulls meeting area constraints
                if (check_hull_area(contourArea(hull[0], false)))
                {
                    hulls.push_back(hull);
                }
            }
        }

        // draw convex hulls on the image
        for (auto hull : hulls)
            drawContours(filtered_line_img, hull, -1, Scalar(0, 255, 0), 2);

        // publish images
        if (_param.debug)
            publish_img_msgs(raw_img, proc_img, line_img, filtered_line_img);
    }
    else
    {
        if (_param.debug)
            ROS_INFO("Not enough lines detected");
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
        Lines filt_lines;
        // filt_lines = filter_lines_by_mid_pts_dist(clustered_lines[i]);
        filt_lines = filter_lines_by_gransac(clustered_lines[i]);
        filt_lines = filter_lines_by_mid_pts_dist(filt_lines);
        processed_lines.push_back(filt_lines);
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

    if (lines.size() > _min_stair_steps)
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

        double sum = std::accumulate(min_dists.begin(), min_dists.end(), 0.0);
        double mean = sum / min_dists.size();

        std::vector<double> diff(min_dists.size());
        std::transform(min_dists.begin(), min_dists.end(), diff.begin(), [mean](double x) { return x - mean; });

        double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / min_dists.size());

        // if (_param.debug)
            // ROS_INFO("Mean = %f | Variance = %f", mean, stdev);

        if (stdev < 20 && mean < 60)
        {
            if (_param.debug)
                ROS_INFO("Potential stair!");
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

void stairDetector::morph_filter(const cv::Mat &img_in, cv::Mat &img_out)
{
    // erosion, dilation filters
    // https://docs.opencv.org/trunk/d4/d86/group__imgproc__filter.html#gaeb1e0c1033e3f6b891a25d0511362aeb
    // dilate(
    //     img_in,
    //     img_out,
    //     Mat(_param.morph_kernel_size, _param.morph_kernel_size, CV_8UC1),
    //     Point(-1, -1),
    //     1,
    //     1,
    //     1);
    // erode(
    //     img_out,
    //     img_out,
    //     Mat(_param.morph_kernel_size, _param.morph_kernel_size, CV_8UC1),
    //     Point(-1, -1),
    //     _param.morph_num_iter,
    //     1,
    //     1);

    Mat element = getStructuringElement(
        0,
        Size(2 * _param.morph_kernel_size + 1, 2 * _param.morph_kernel_size + 1),
        Point(_param.morph_kernel_size, _param.morph_kernel_size));

    // morphologyEx(img_in, img_out, MORPH_CLOSE, element);
    morphologyEx(img_in, img_out, MORPH_OPEN, element, Point(-1, -1), _param.morph_num_iter);
    morphologyEx(img_out, img_out, MORPH_ERODE, element, Point(-1, -1), _param.morph_num_iter);
    // morphologyEx(img_in, img_out, MORPH_CLOSE, element);
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
            circle(image, ipt, 8, _color_tab[j], 2, 8);
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

    for (auto it : xy_lines)
    {
        Line line(it);
        line.calPixels(img_in);
        lines.push_back(line);
    }
    if (_param.debug)
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
    Mat points, labels, centers;

    for (auto line : lines)
    {
        points.push_back(line.p_mid);
    }

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

Lines stairDetector::filter_lines_by_gransac(const Lines &lines_in)
{
    vector<Point> mid_points;
    Vec4f inliers;
    Lines lines_out;

    for (auto r : lines_in)
        mid_points.push_back(r.p_mid);

    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
    for (int i = 0; i < lines_in.size(); ++i)
    {
        Point Pt(lines_in[i].p_mid.x, lines_in[i].p_mid.y);

        std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point2D>(Pt.x, Pt.y);
        CandPoints.push_back(CandPt);
    }

    GRANSAC::RANSAC<Line2DModel, 2> Estimator;
    Estimator.Initialize(10, 100); // Threshold, iterations
    Estimator.Estimate(CandPoints);

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

            Point2f Pt(RPt->m_Point2D[0], RPt->m_Point2D[1]);
            for (auto line : lines_in)
            {
                if (line.p_mid == Pt)
                {
                    lines_out.push_back(line);
                }
            }
        }
    }
    else
    {
        // if (_param.debug)
        if (false)
        {
            ROS_INFO("Not enough inliers in to run ransac");
            ROS_INFO("Size Lines IN = %d", lines_in.size());
            ROS_INFO("Size Lines OUT = %d", lines_out.size());
        }
    }
    return lines_out;
}

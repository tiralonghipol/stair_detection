#include "stairdetect/stairdetect.h"

stairDetector::stairDetector(ros::NodeHandle &n, const std::string &s, int bufSize)
{
    // get params from .launch file
    // topics
    n.getParam("topic_stitched_pointcloud", _topic_stitched_pcl);
    n.getParam("topic_pose", _topic_pose);
    n.getParam("topic_trimmed_pcl", _topic_trimmed_pcl);
    n.getParam("topic_bird_eye_img", _topic_bird_eye_img);
    n.getParam("stair_detect_timing", _stair_detect_time);

    // pointcloud trimming parameters
    _xy_lim = 10;
    _z_lim = 1;

    // other params
    _pose_Q_size = 40;

    // initialize recent pointcloud
    // const pcl::PointCloud<pcl::PointXYZ>::Ptr init_pcl;
    // _recent_cloud = nullptr;

    // test flag
    n.getParam("test_mode", _param.debug);

    // subscribers
    _sub_stitched_pcl = n.subscribe(
        _topic_stitched_pcl, 1, &stairDetector::callback_stitched_pcl, this);
    _sub_pose = n.subscribe(
        _topic_pose, 50, &stairDetector::callback_pose, this);

    // publishers
    _pub_trimmed_pcl = n.advertise<sensor_msgs::PointCloud2>(_topic_trimmed_pcl, bufSize);
    image_transport::ImageTransport it(n);
    _pub_bird_view_img = it.advertise(_topic_bird_eye_img, 1);

    // timers
    _timer_stair_detect = n.createTimer(
        ros::Duration(_stair_detect_time), &stairDetector::callback_timer_trigger, this);
}

void stairDetector::callback_stitched_pcl(
    // const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & msg)
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
    // std::clock_t c_start = std::clock();
    pcl_to_bird_view_img(_recent_cloud, img);
    // std::clock_t c_end = std::clock();
    // std::cout << "bime in birds-eye image construction: " << (c_end - c_start)/CLOCKS_PER_SEC << std::endl;

    // image processing....
    // stairDetector::filter_img();
    return;
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

    // make and publish message
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    _pub_bird_view_img.publish(img_msg);
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
}

// void filter_img(const cv::Mat &bird_view_img)
void stairDetector::filter_img()
{
    Mat image;
    if (_param.debug)
    {
        // Some code
        ROS_INFO_ONCE("Before try");
        try
        {
            image = imread("/home/pol/stair_ws/src/stairdetect/imgs_test/top_view_matlab.pgm", CV_LOAD_IMAGE_GRAYSCALE);
            ROS_INFO_ONCE("Inside try");
            if (image.empty())
            {
                throw image;
                ROS_INFO_ONCE("After throw (Never executed)");
            }

            imshow("Input Image", image); // Show our image inside it.
            // waitKey(0);                   // Wait for a keystroke in the window
        }
        catch (Mat image)
        {
            ROS_WARN_ONCE("Exception Caught \n");
        }

        ROS_WARN_ONCE("After catch (Will be executed) \n");
    }
    return;
}

void stairDetector::cannyEdgeDetection(const cv::Mat &input_image, cv::Mat &edge)
{
    /// Reduce noise with a kernel 3x3
    cv::blur(input_image, edge, cv::Size(3, 3));
    /// Canny detector
    cv::Canny(edge, edge, (double)_param.canny_low_th, (double)_param.canny_low_th * _param.canny_ratio, _param.canny_kernel_size);
}

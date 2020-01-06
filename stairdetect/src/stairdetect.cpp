#include "stairdetect/stairdetect.h"

stairDetector::stairDetector(ros::NodeHandle &n, const std::string &s, int bufSize)
{
    // get params from .launch file
    // topics
    n.getParam("topic_stitched_pointcloud", _topic_stitched_pcl);
    n.getParam("topic_pose", _topic_pose);
    n.getParam("topic_trimmed_pcl", _topic_trimmed_pcl);

    // pointcloud trimming parameters
    _xy_lim = 10;
    _z_lim = 1;

    // other params
    _pose_Q_size = 40;

    // test flag
    n.getParam("test_mode", _test_mode);

    // publishers
    _pub_trimmed_pcl = n.advertise<sensor_msgs::PointCloud2>(_topic_trimmed_pcl, bufSize);

    // subscribers
    _sub_stitched_pcl = n.subscribe(
        _topic_stitched_pcl, 50, &stairDetector::callback_stitched_pcl, this);
    _sub_pose = n.subscribe(
        _topic_pose, 50, &stairDetector::callback_pose, this);
}

void stairDetector::callback_stitched_pcl(
    // const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & msg)
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
{
    if (_test_mode)
    {
        ROS_INFO("in stitched pointcloud callback...");
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

void filter_img(const cv::Mat &bird_view_img)
{
    // cv::Mat image = imread("imgs_test/top_view_matlab.pgm", CV_LOAD_IMAGE_COLOR);

    return;
}

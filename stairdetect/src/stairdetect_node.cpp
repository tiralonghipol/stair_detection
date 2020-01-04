#include "ros/ros.h"
#include "stairdetect/stairdetect.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stairdetect");
    ros::NodeHandle nh("stairdetect");

    stairDetector sd = stairDetector(nh, "stair_detector_node", 100);
    ros::spin();
    return 0;
}
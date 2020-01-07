#include "ros/ros.h"
#include "stairdetect/stairdetect.h"
#include <boost/bind.hpp>

// #include <dynamic_reconfigure/server.h>
// #include <stairdetect/StairDetectConfig.h>

stairDetectorParams param;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stairdetect");
    ros::NodeHandle nh("stairdetect");

    // stairDetector sd = stairDetector(nh, "stair_detector_node", 100);
    stairDetector *sd = new stairDetector(nh, "stair_detector_node", 100);

    param.debug = true;
    sd->setParam(param);

    
    
    
    // dynamic_reconfigure::Server<stairdetect::StairDetectConfig> dr_srv;

    // dynamic_reconfigure::Server<stairdetect::StairDetectConfig>::CallbackType cb;
    // cb = boost::bind(&stairDetector::callback_dyn_reconf, sd, _1, _2);
    // dr_srv.setCallback(cb);
    
    Mat tmp;
    sd->filter_img(tmp);

    ros::spin();
    return 0;
}
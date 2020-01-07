#include "ros/ros.h"
#include "stairdetect/stairdetect.h"


#include <dynamic_reconfigure/server.h>
#include <stairdetect/StairDetectConfig.h>

stairDetectorParams param;

// void callback_dyn_reconf(stairdetect::StairDetectConfig &config, uint32_t level)
// {
//     // high level bools
//     param.debug = config.debug;
//     // canny
//     param.canny_low_th = config.canny_low_th;
//     param.canny_ratio = config.canny_ratio;
//     param.canny_kernel_size = config.canny_kernel_size;

//     // sd.setParam(param);
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stairdetect");
    ros::NodeHandle nh("stairdetect");

    stairDetector sd = stairDetector(nh, "stair_detector_node", 100);
    param.debug = true;
    sd.setParam(param);

    // dynamic_reconfigure::Server<stairdetect::StairDetectConfig> dsrv(ros::NodeHandle("~"));
    // dynamic_reconfigure::Server<stairdetect::StairDetectConfig>::CallbackType cb;
    // cb = boost::bind(&callback_dyn_reconf, _1, _2);
    // dsrv.setCallback(cb);

    ros::spin();
    return 0;
}
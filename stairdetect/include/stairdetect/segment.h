#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/angles.h"
#include "pcl/common/common.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/statistical_outlier_removal.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

class Segmenter
{
public:
    Segmenter(const ros::Publisher &surface_points_pub,
              const ros::Publisher &marker_pub,
              const ros::Publisher &above_surface);

    void segment_surface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                         pcl::PointIndices::Ptr indices);

    void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                   geometry_msgs::Pose *pose,
                                   geometry_msgs::Vector3 *dimensions);

private:
    ros::Publisher _surface_points_pub;
    ros::Publisher _marker_pub;
    ros::Publisher _above_surface_pub;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr _actual_cloud;
};
// added to check git
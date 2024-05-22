#ifndef PC_PROCESSING_BA
#define PC_PROCESSING_BA

#pragma region includes

#include "sensor_msgs/PointCloud2.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ba_types.h"

#pragma endregion

#pragma region defines

#define BOX_SIZE 0.15
// #define BA_PCL_DEBUG

#pragma endregion

/**
 * @brief Functions regarding the pointcloud processing
 * 
 */
namespace pointcloud_processing
{

    TomatoCoordinates FitSphereInCloud(const sensor_msgs::PointCloud2 &input, pcl::PointXYZ roi, bool reset_rviz_cloud = false);

    void CropPointCloud(BAPointCloudPtr input, BAPointCloudPtr output, pcl::PointXYZ roi);

    void SegregatePlane(BAPointCloudPtr input, BAPointCloudPtr output);

    TomatoCoordinates FitSphere(BAPointCloudPtr input);
}

#endif
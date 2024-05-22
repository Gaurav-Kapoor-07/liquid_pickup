#ifndef BA_TYPES_H
#define BA_TYPES_H

#pragma region includes

#include <cmath>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/sample_consensus/sac_model_sphere.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/passthrough.h>

#pragma endregion

// typedef pcl::PointCloud<pcl::PointXYZ>::Ptr BAPointCloudPtr;
// typedef pcl::PointCloud<pcl::PointXYZ> BAPointCloud;

/**
 * @brief Struct to store tomato coordinates
 * 
 */
struct TomatoCoordinates{
    float x;
    float y;
    float z;
    float r;
    bool is_picked;
    inline bool operator==(const TomatoCoordinates t1){
        return (pow(x-t1.x,2)+pow(y-t1.y,2)+pow(z-t1.z,2))<std::max(r, t1.r)+0.01;
    };
} ;

/**
 * @brief Struct to specify single locations of the path where the robot has to drive.
 * 
 */
struct PathLocation {
    float x;
    float y;
    float orientation;
};

/**
 * @brief Enum to specify whether to push/pop in front or back of queue.
 * 
 */
enum QueueSide {
    front,
    back
};

#endif
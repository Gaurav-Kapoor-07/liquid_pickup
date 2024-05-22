#ifndef BA_IMAGE_PROCESSING_H
#define BA_IMAGE_PROCESSING_H

#pragma region includes

#include <pcl/point_types.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

#pragma endregion

/**
 * @brief Functions and types regarding the image processing
 * 
 */
namespace image_processing
{
    #pragma region typedefs, variables etc.

    typedef struct
    {
        cv::Scalar lower_bounds; // HSV values H[0,179] S[0,255] V[0,288]
        cv::Scalar upper_bounds;
    } ColorThreshold;

    typedef std::map<std::string, ColorThreshold> ColorThresholdMap;

    const ColorThreshold kRed1 = {cv::Scalar(0, 160, 10), cv::Scalar(15, 255, 255)};
    const ColorThreshold kRed2 = {cv::Scalar(165, 160, 10), cv::Scalar(180, 255, 255)};

    const ColorThresholdMap hsv_threshold_dict = {
        {"kRed1", kRed1},
        {"kRed2", kRed2}};

    #pragma endregion

    #pragma region methods

    pcl::PointXYZ GetROICenter(cv::KeyPoint tomato, sensor_msgs::PointCloud2 pcloud);

    void PerceptFilterThreshold(std::string target_color,
                                cv::Mat &img_hsv,
                                cv::Mat &img_threshold);

    std::vector<cv::KeyPoint> PerceptBlobDetection(cv::Mat &img,
                                                   cv::Mat &img_threshold,
                                                   cv::Mat &img_keypoints,
                                                   cv::SimpleBlobDetector::Params params);

    #pragma endregion
}

#endif
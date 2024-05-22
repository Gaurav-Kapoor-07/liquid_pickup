#include "image_processing.h"

/**
 * @brief Gets the coordinates of the center of the region of interets (tomato)
 * 
 * @param tomato The tomato to analyze
 * @param pcloud The pointcloud information
 * @return pcl::PointXYZ The region of interest center
 */
pcl::PointXYZ image_processing::GetROICenter(cv::KeyPoint tomato, sensor_msgs::PointCloud2 pcloud){
    int32_t pointstep = pcloud.point_step;
    int32_t rowstep = pcloud.row_step;
    pcl::PointXYZ result;
    int32_t idx_x = ((int)tomato.pt.y) * rowstep + ((int)tomato.pt.x) * pointstep;
    result.x = *(float *)&pcloud.data[idx_x];
    result.y = *(float *)&pcloud.data[idx_x + 4];
    result.z = *(float *)&pcloud.data[idx_x + 8];
    return result;
}

/**
 * @brief Filters HSV image according to target_color defined in header (hsv_threshold_dict). creates binary mask where pixels within the target_color range are white and black otherwise
 * 
 * @param target_color String to specify which color to load from hsv_threshold_dict defined in header
 * @param img_hsv color image in hsv color space
 * @param img_threshold output image after filtering
 */
void image_processing::PerceptFilterThreshold(std::string target_color, cv::Mat &img_hsv, cv::Mat &img_threshold)
{
    if (target_color.compare("red") == 0)
    {
        cv::Mat img_threshold_2;
        ColorThreshold th_1 = hsv_threshold_dict.at("kRed1");
        ColorThreshold th_2 = hsv_threshold_dict.at("kRed2");
        cv::inRange(img_hsv,
                    th_1.lower_bounds,
                    th_1.upper_bounds, img_threshold);
        cv::inRange(img_hsv,
                    th_2.lower_bounds,
                    th_2.upper_bounds, img_threshold_2);
        img_threshold = img_threshold | img_threshold_2;
    }
    else
    {
        ColorThreshold th = hsv_threshold_dict.at(target_color);
        cv::inRange(img_hsv,
                    th.lower_bounds,
                    th.upper_bounds, img_threshold);
    }
}

/**
 * @brief Filters a binary image for blobs with size specified in params parameter
 * 
 * @param img input image
 * @param img_threshold input mask
 * @param img_keypoints output image with keypoints visualized over original image
 * @param params parameter object for blobdetection (e.g. min/max size)
 * @return std::vector<cv::KeyPoint> 
 */
std::vector<cv::KeyPoint> image_processing::PerceptBlobDetection(cv::Mat &img, cv::Mat &img_threshold, cv::Mat &img_keypoints, cv::SimpleBlobDetector::Params params)
{
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(img_threshold, keypoints);
    cv::drawKeypoints(img, keypoints, img_keypoints,
                      cv::Scalar(250, 0, 0),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


    return keypoints;
};
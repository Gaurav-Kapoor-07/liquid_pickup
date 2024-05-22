#pragma region includes

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "pointcloud_processing.h"

#include "manipulator.h"
#include "tomato_queue.h"
#include "image_processing.h"

#include "ba_frames_summit_xl.h"
#include "ba_interfaces.h"

#pragma endregion

#pragma region defines

#define BLOB_MIN_PX 200
#define BLOB_MAX_PX 5000
#define VISION_SAVE_IMG
#define VISION_SHOW_IMG

#include "time_logger.h"
#ifdef TIME_LOGGER_ON
#define LOG_VISION_START(val) BATimeLogger::LogImageProcessing(val, log_start)
#define LOG_VISION_STOP(val) BATimeLogger::LogImageProcessing(val, log_stop)
#define LOG_POSE_SCAN(ps, t) BATimeLogger::LogScan(ps, t)
#define LOG_POSE_VAL(ps, t) BATimeLogger::LogValidate(ps, t)
#else
#define LOG_VISION_START(val)
#define LOG_VISION_STOP(val)
#define LOG_POSE_SCAN(ps,t) 
#define LOG_POSE_VAL(ps,t) 
#endif


#pragma endregion

geometry_msgs::PoseStamped EstimateTomatoLocation(TomatoCoordinates *tomato, sensor_msgs::PointCloud2 latest_cloud2_);

#pragma region LookForObject

/**
 * @brief Look for an object of a particular color
 *
 */
class LookForObject : public BT::ConditionNode, public IBAInitManipulatorNode, public IBAInitNodeHandle, public IBAInitTomatoQueue
{
public:
  LookForObject(const std::string &name);
  void init(ros::NodeHandle node_handle) override;
  void init(Manipulator manipulator) override;
  void init(TomatoQueue &tomato_queue) override;
  BT::NodeStatus tick() override;

private:
  ros::NodeHandle node_handle_;
  Manipulator manipulator_;
  TomatoQueue *tomato_queue_;
  bool received_image_;
  bool received_cloud2_;
  sensor_msgs::ImageConstPtr latest_image_;
  sensor_msgs::PointCloud2 latest_cloud2_;
  std::string target_color;
  std::string img_path;
  std::string img_temp;
  cv::SimpleBlobDetector::Params params;
  void ImageCallback(const sensor_msgs::ImageConstPtr &msg);
  void CloudCallback2(const sensor_msgs::PointCloud2 &cloud);
  void FetchNewPerception(void);
};

#pragma endregion

#pragma region ValidateTomato

/**
 * @brief Class/Behavior to validate if a found object is a tomato
 * 
 */
class ValidateTomato : public BT::ConditionNode, public IBAInitNodeHandle
{
public:
  ValidateTomato(const std::string &name, const BT::NodeConfiguration &config);
  void init(ros::NodeHandle nh_) override;
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts(void);

private:
  ros::NodeHandle node_handle_;
  TomatoQueue *tomato_queue_;
  bool received_image_;
  bool received_cloud2_;
  sensor_msgs::ImageConstPtr latest_image_;
  sensor_msgs::PointCloud2 latest_cloud2_;
  std::string target_color;
  std::string img_path;
  std::string img_temp;
  cv::SimpleBlobDetector::Params params;
  void ImageCallback(const sensor_msgs::ImageConstPtr &msg);
  void CloudCallback2(const sensor_msgs::PointCloud2 &cloud);
  void FetchNewPerception(void);
};

#pragma endregion
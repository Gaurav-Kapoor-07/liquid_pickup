// Vision related behaviors

#include "vision_behaviors.h"

/* #ifdef TIME_LOGGER_ON
void LOGSTART(std::string val){ROS_WARN("LOGGGGING");BATimeLogger::LogImageProcessing(val, log_start);}
void LOGEND(std::string val){BATimeLogger::LogImageProcessing(val, log_stop);}
#else
void LOGSTART(std::string val){ROS_WARN("NOT LOGGGGING");}
void LOGEND(std::string val){}
#endif */

#pragma region LookForObject

#pragma region public

/**
 * @brief Construct a new Look For Object:: Look For Object object
 *
 * @param name The name of the node
 */
LookForObject::LookForObject(const std::string &name) : BT::ConditionNode(name, {})
{
    ros::param::get("target_color", target_color);
    ros::param::get("img_path", img_path);
    ros::param::get("img_temp", img_temp);
    params.minArea = BLOB_MIN_PX;
    params.maxArea = BLOB_MAX_PX;
    params.filterByArea = true;
    params.filterByColor = false;
    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.thresholdStep = 50;
    ROS_INFO("[%s] Initialized.", this->name().c_str());
}

/**
 * @brief Initializes the NodeHandle
 *
 * @param tomato_queue The NodeHandle
 */
void LookForObject::init(ros::NodeHandle node_handle)
{
    node_handle_ = node_handle;
}

/**
 * @brief Initializes the Manipulator
 *
 * @param manipulator The Manipulator
 */
void LookForObject::init(Manipulator manipulator)
{
    manipulator_ = manipulator;
}

/**
 * @brief Initializes the reference to the TomatoQueue
 *
 * @param tomato_queue The reference to the TomatoQueue
 */
void LookForObject::init(TomatoQueue &tomato_queue)
{
    tomato_queue_ = &tomato_queue;
}

/**
 * @brief Handles the tick from the behavior tree
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus LookForObject::tick()
{
    LOG_VISION_START(this->name());
    ROS_INFO("[%s] Looking for %s object.", this->name().c_str(), target_color.c_str());
    FetchNewPerception(); // Receive an image
    // Convert to HSV and threshold
    cv::Mat img, img_hsv, img_threshold, img_keypoints;
    img = cv_bridge::toCvShare(latest_image_, "bgr8")->image;
    cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
    image_processing::PerceptFilterThreshold(target_color, img_hsv, img_threshold);
    // Do blob detection
    std::vector<cv::KeyPoint> keypoints = image_processing::PerceptBlobDetection(img, img_threshold, img_keypoints, params);
// Display the image
#ifdef VISION_SHOW_IMG
    if (!img_keypoints.empty())
    {
        cv::imwrite(img_temp + "raw.jpg", img);
        cv::imwrite(img_temp + "keypoints.jpg", img_keypoints);
        cv::imwrite(img_temp + "threshold.jpg", img_threshold);
    }
#endif

// Save Image
#ifdef VISION_SAVE_IMG
    static int img_cntr = 1;
    std::string filename = img_path + "img" + std::to_string(img_cntr) + ".jpg";
    img_cntr++;
    cv::imwrite(filename, img_keypoints);
#endif
    if (keypoints.size() > 0)
    {
        ROS_INFO("---------------I found some tomatoes!---------------");
        uint8_t tomato_counter = 1;

        for (auto tomato : keypoints)
        {
            ROS_INFO("TOMATO %d @ (%.2f, %.2f) with radius %.2f px.", tomato_counter, tomato.pt.x, tomato.pt.y, tomato.size);
            pcl::PointXYZ center_of_ROI = image_processing::GetROICenter(tomato, latest_cloud2_);
            TomatoCoordinates t = pointcloud_processing::FitSphereInCloud(latest_cloud2_, center_of_ROI, false);
            geometry_msgs::PoseStamped tomato_map = EstimateTomatoLocation(&t, latest_cloud2_);
            LOG_POSE_SCAN(tomato_map,t.r);
            tomato_queue_->AddTomato(t);
        }
        tomato_queue_->PrintQueue();
        LOG_VISION_STOP(this->name());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        ROS_WARN("[%s] No object detected!", this->name().c_str());
        LOG_VISION_STOP(this->name());
        return BT::NodeStatus::FAILURE;
    }
}

#pragma endregion

#pragma region private

/**
 * @brief subscribe to rgb image and depth pointcloud
 *
 */
void LookForObject::FetchNewPerception()
{
    image_transport::ImageTransport it(node_handle_);
    image_transport::Subscriber sub = it.subscribe("/summit_xl/arm_camera/rgb/image_raw", 1, &LookForObject::ImageCallback, this);

    ros::Subscriber sub_cloud_2 = node_handle_.subscribe("/summit_xl/arm_camera/depth_registered/points", 1, &LookForObject::CloudCallback2, this);
    received_image_ = false;
    received_cloud2_ = false;
    ros::Duration(1.0).sleep();
    while ((!received_image_) || (!received_cloud2_))
    {
        ros::spinOnce();
    }
    sub.shutdown();
    sub_cloud_2.shutdown();
}

/**
 * @brief Estimates the manipulator pose from the TomatoCoordinates and the pointcloud
 * 
 * @param tomato_coordinates The coordinates of the tomato
 * @param latest_cloud2_ The latest pointcloud
 * @return geometry_msgs::PoseStamped The pose
 */
geometry_msgs::PoseStamped EstimateTomatoLocation(TomatoCoordinates *tomato_coordinates, sensor_msgs::PointCloud2 latest_cloud2_)
{
    tf::TransformListener listener;
    listener.waitForTransform(MAP_FRAME, CAMERA_FRAME, ros::Time(0), ros::Duration(3.0));
    geometry_msgs::PoseStamped tomato_cameraframe;
    geometry_msgs::PoseStamped tomato_mapframe;
    tomato_cameraframe.pose.position.x = tomato_coordinates->x;
    tomato_cameraframe.pose.position.y = tomato_coordinates->y;
    tomato_cameraframe.pose.position.z = tomato_coordinates->z;
    tomato_cameraframe.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 1.5708);
    tomato_cameraframe.header.frame_id = latest_cloud2_.header.frame_id;
    tomato_cameraframe.header.stamp = ros::Time();
    listener.transformPose(MAP_FRAME, tomato_cameraframe, tomato_mapframe);
    tomato_mapframe.header.frame_id = MAP_FRAME;
    tomato_mapframe.header.stamp = ros::Time();
    tomato_coordinates->x = tomato_mapframe.pose.position.x;
    tomato_coordinates->y = tomato_mapframe.pose.position.y;
    tomato_coordinates->z = tomato_mapframe.pose.position.z;
    return tomato_mapframe;
}

void LookForObject::ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    latest_image_ = msg;
    received_image_ = true;
}

void LookForObject::CloudCallback2(const sensor_msgs::PointCloud2 &cloud)
{
    latest_cloud2_ = cloud;
    received_cloud2_ = true;
}

#pragma endregion

#pragma endregion

#pragma region ValidateTomate

#pragma region public

/**
 * @brief Construct a new Validate Tomato:: Validate Tomato object
 * 
 * @param name 
 * @param config 
 */
ValidateTomato::ValidateTomato(const std::string &name, const BT::NodeConfiguration &config) : BT::ConditionNode(name, config)
{
    ros::param::get("target_color", target_color);
    ros::param::get("img_path", img_path);
    ros::param::get("img_temp", img_temp);
    params.minArea = BLOB_MIN_PX;
    params.maxArea = BLOB_MAX_PX;
    params.filterByArea = true;
    params.filterByColor = false;
    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.thresholdStep = 50;
    ROS_INFO("[%s] Initialized.", this->name().c_str());
}

/**
 * @brief Initializes reference to the NodeHandle
 *
 * @param node_handle Reference to the NodeHandle
 */
void ValidateTomato::init(ros::NodeHandle node_handle)
{
    node_handle_ = node_handle;
}

/**
 * @brief Handles the tick from the behavior tree
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ValidateTomato::tick()
{
    LOG_VISION_START(this->name());
    ROS_INFO("[%s] Looking for %s object.", this->name().c_str(), target_color.c_str());
    FetchNewPerception(); // Receive an image
    // Convert to HSV and threshold
    cv::Mat img, img_hsv, img_threshold, img_keypoints;
    img = cv_bridge::toCvShare(latest_image_, "bgr8")->image;
    cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
    image_processing::PerceptFilterThreshold(target_color, img_hsv, img_threshold);
    // Do blob detection
    std::vector<cv::KeyPoint> keypoints = image_processing::PerceptBlobDetection(img, img_threshold, img_keypoints, params);
    // Display the image
#ifdef VISION_SHOW_IMG
    
    cv::imwrite(img_temp + "keypoints.jpg", img_keypoints);
    cv::imwrite(img_temp + "threshold.jpg", img_threshold);
#endif
    float min_distnace_img_center = FLT_MAX;
    float center_x = img_keypoints.size().width / 2.0f;
    float center_y = img_keypoints.size().height / 2.0f;
    cv::KeyPoint center;
    if (keypoints.size() > 0)
    {
        for (auto kp : keypoints)
        {
            float center_dist = pow(kp.pt.x - center_x, 2) + pow(kp.pt.y - center_y, 2);
            if (center_dist < min_distnace_img_center)
            {
                min_distnace_img_center = center_dist;
                center = kp;
            }
        }
        ROS_INFO("%.2f %.2f", center.pt.x, center.pt.y);
        pcl::PointXYZ center_of_ROI = image_processing::GetROICenter(center, latest_cloud2_);
        TomatoCoordinates t = pointcloud_processing::FitSphereInCloud(latest_cloud2_, center_of_ROI, true);
        geometry_msgs::PoseStamped tomato_map = EstimateTomatoLocation(&t, latest_cloud2_);
        LOG_POSE_VAL(tomato_map, t.r);
        BT::Optional<float> tomato_map_x = getInput<float>("target_x");
        BT::Optional<float> tomato_map_y = getInput<float>("target_y");
        BT::Optional<float> tomato_map_z = getInput<float>("target_z");
        double delta = sqrt(pow(tomato_map.pose.position.x - tomato_map_x.value(), 2) +
                pow(tomato_map.pose.position.y - tomato_map_y.value(), 2) +
                pow(tomato_map.pose.position.z - tomato_map_z.value(), 2));
        if (delta < 0.07)
        {
            float airgap = ba_helper::CalculateGripperParameterFromDesiredAirgap(2 * t.r);
            ROS_INFO("Desired airgap between gripper fingers: %.3f m", 2*t.r);
            setOutput("gripper_airgap", (float)airgap);
            setOutput("target_x", (float)tomato_map.pose.position.x);
            setOutput("target_y", (float)tomato_map.pose.position.y);
            setOutput("target_z", (float)tomato_map.pose.position.z);
            LOG_VISION_STOP(this->name());
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("Delta: %.4f", delta);
            setOutput("gripper_airgap", 0.0f);
            LOG_VISION_STOP(this->name());
            return BT::NodeStatus::FAILURE;
        }
    }
    else
    {
        LOG_VISION_STOP(this->name());
        return BT::NodeStatus::FAILURE;
    }
}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ValidateTomato::providedPorts()
{
    return {BT::InputPort<float>("target_x"),
            BT::InputPort<float>("target_y"),
            BT::InputPort<float>("target_z"),
            BT::OutputPort<float>("target_x"),
            BT::OutputPort<float>("target_y"),
            BT::OutputPort<float>("target_z"),
            BT::OutputPort<float>("gripper_airgap")};
}


#pragma endregion

#pragma region private

/**
 * @brief subscribe to rgb image and depth pointcloud
 *
 */
void ValidateTomato::FetchNewPerception()
{
    image_transport::ImageTransport it(node_handle_);
    image_transport::Subscriber sub = it.subscribe("/summit_xl/arm_camera/rgb/image_raw", 1, &ValidateTomato::ImageCallback, this);

    ros::Subscriber sub_cloud_2 = node_handle_.subscribe("/summit_xl/arm_camera/depth_registered/points", 1, &ValidateTomato::CloudCallback2, this);
    received_image_ = false;
    received_cloud2_ = false;
    ros::Duration(1.0).sleep();
    while ((!received_image_) || (!received_cloud2_))
    {
        ros::spinOnce();
    }
    sub.shutdown();
    sub_cloud_2.shutdown();
}

/**
 * @brief Callback method regarding image information
 * 
 * @param msg The reference to the ImageConstPtr 
 */
void ValidateTomato::ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    latest_image_ = msg;
    received_image_ = true;
}

/**
 * @brief Callback method regarding point cloud information
 * 
 * @param cloud The reference to the PointCloud2
 */
void ValidateTomato::CloudCallback2(const sensor_msgs::PointCloud2 &cloud)
{
    latest_cloud2_ = cloud;
    received_cloud2_ = true;
}

#pragma endregion

#pragma endregion
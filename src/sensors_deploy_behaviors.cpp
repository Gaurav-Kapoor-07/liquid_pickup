#include "sensors_deploy_behaviors.h"

#pragma region SensorsDeploy

#pragma region public methods
/**
 * @brief Construct a new SensorsDeploy:: SensorsDeploy object
 *
 * @param name The name of the behavior
 * @param config The node configuration
 */
SensorsDeploy::SensorsDeploy(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node) : BT::SyncActionNode(name, config)
{
    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s] Node shared pointer was passed!", this->name().c_str());
    }

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    deploy_sensor_names_publisher_ = node_->create_publisher<std_msgs::msg::String>("/summit/deploy_sensor_names", 10);
    
    RCLCPP_INFO(node_->get_logger(), "[%s] Initialized!", this->name().c_str());
}

/**
 * @brief Handles the tick from the behavior tree
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus SensorsDeploy::tick()
{
    // Format for sensor poses: sensor_1_name,sensor_1_x,sensor_1_y,;sensor_2_name,sensor_2_x,sensor_2_y,;sensor_3_name,sensor_3_x,sensor_3_y,;
    
    std::string sensor_poses_;
    getInput<std::string>("sensor_poses", sensor_poses_);

    unsigned int no_of_deploy_sensors{0};
    
    auto sensor_names_msg = std_msgs::msg::String();
    
    while (!sensor_poses_.empty())
    {
        std::size_t pos_semicolon = sensor_poses_.find(";");
        std::string sensor_poses_substr = sensor_poses_.substr(0, pos_semicolon);

        std::vector<std::string> pose_xy_vector;
        unsigned int i{0};

        while (!sensor_poses_substr.empty())
        {
            std::size_t pos_comma = sensor_poses_substr.find(",");

            pose_xy_vector.push_back(sensor_poses_substr.substr(0, pos_comma));
            
            sensor_poses_substr.erase(0, pos_comma + 1);
            i += 1;
        }

        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = node_->get_clock()->now();
        t.header.frame_id = PORT_FRAME;
        t.child_frame_id = pose_xy_vector.at(0);
        t.transform.translation.x = std::stod(pose_xy_vector.at(1));
        t.transform.translation.y = std::stod(pose_xy_vector.at(2));
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        sensor_names_msg.data += t.child_frame_id;
        sensor_names_msg.data += ",";

        // Send the transformation
        tf_broadcaster_->sendTransform(t);

        no_of_deploy_sensors += 1;

        sensor_poses_.erase(0, pos_semicolon + 1);
    }

    deploy_sensor_names_publisher_->publish(sensor_names_msg);

    setOutput("no_of_deploy_sensors", no_of_deploy_sensors);
    
    RCLCPP_INFO(node_->get_logger(), "all %d sensor poses parsed!", no_of_deploy_sensors); 
    
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList SensorsDeploy::providedPorts()
{
    return {BT::InputPort<std::string>("sensor_poses"), BT::OutputPort<int>("no_of_deploy_sensors")};
}

#pragma endregion

#pragma endregion

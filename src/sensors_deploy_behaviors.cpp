#include "sensors_deploy_behaviors.h"

#pragma region SensorsDeploy

#pragma region public methods
/**
 * @brief Construct a new SensorsDeploy:: SensorsDeploy object
 *
 * @param name The name of the behavior
 * @param config The node configuration
 */
SensorsDeploy::SensorsDeploy(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node) : BT::StatefulActionNode(name, config)
{
    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s] Node shared pointer was passed!", this->name().c_str());
    }
    
    RCLCPP_INFO(node_->get_logger(), "[%s] Initialized!", this->name().c_str());
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus SensorsDeploy::onStart()
{
    // Format for sensor_deploy_frame_names: sensor_1_name,sensor_2_name,sensor_3_name,
    
    std::string sensor_deploy_frame_names_;
    getInput<std::string>("sensor_deploy_frame_names", sensor_deploy_frame_names_);
    setOutput<std::string>("sensor_deploy_frame_names_dynamic", sensor_deploy_frame_names_);
    
    int no_of_deploy_sensors = std::count(sensor_deploy_frame_names_.begin(), sensor_deploy_frame_names_.end(), ',');

    setOutput<int>("no_of_deploy_sensors", no_of_deploy_sensors);
    
    RCLCPP_INFO(node_->get_logger(), "%d sensors to be deployed!", no_of_deploy_sensors); 
    
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus SensorsDeploy::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 *
 */
void SensorsDeploy::onHalted(){};

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList SensorsDeploy::providedPorts()
{
    return {BT::InputPort<std::string>("sensor_deploy_frame_names"), BT::OutputPort<std::string>("sensor_deploy_frame_names_dynamic"), BT::OutputPort<int>("no_of_deploy_sensors")};
}

#pragma endregion

#pragma endregion

#include "manipulator_behaviors.h"

#pragma region ManipulatorGrasp

/**
 * @brief Construct a new Manipulator Grasp :: Manipulator Grasp object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */

ManipulatorGrasp::ManipulatorGrasp(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
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
BT::NodeStatus ManipulatorGrasp::onStart()
{
    LOG_MANI_START(this->name());

    BT::Optional<std::string> action = getInput<std::string>("action");
    BT::Optional<double> base_footprint_x = getInput<double>("target_x");
    BT::Optional<double> base_footprint_y = getInput<double>("target_y");
    BT::Optional<double> base_footprint_z = getInput<double>("target_z");
    BT::Optional<double> base_footprint_roll = getInput<double>("target_roll");
    BT::Optional<double> base_footprint_pitch = getInput<double>("target_pitch");
    BT::Optional<double> base_footprint_yaw = getInput<double>("target_yaw");

    RCLCPP_INFO(node_->get_logger(), "moving gripper to target");
    manipulator_.MoveGripperToPoseLinear(action.value(), base_footprint_x.value(), base_footprint_y.value(), base_footprint_z.value(), base_footprint_roll.value(), base_footprint_pitch.value(), base_footprint_yaw.value());
    RCLCPP_INFO(node_->get_logger(), "moved gripper to target");
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorGrasp::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorGrasp::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorGrasp::providedPorts()
{
    return {BT::InputPort<std::string>("action"), BT::InputPort<double>("target_x"), BT::InputPort<double>("target_y"), BT::InputPort<double>("target_z"), BT::InputPort<double>("target_roll"), BT::InputPort<double>("target_pitch"), BT::InputPort<double>("target_yaw")};
}

#pragma endregion

#pragma region ManipulatorPregrasp

/**
 * @brief Construct a new Manipulator Pregrasp:: Manipulator Pregrasp object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */
ManipulatorPregrasp::ManipulatorPregrasp(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
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
BT::NodeStatus ManipulatorPregrasp::onStart()
{
    LOG_MANI_START(this->name());

    BT::Optional<bool> pose_from_tf = getInput<bool>("pose_from_tf");
    BT::Optional<double> base_footprint_x = getInput<double>("target_x");
    BT::Optional<double> base_footprint_y = getInput<double>("target_y");
    BT::Optional<double> base_footprint_z = getInput<double>("target_z");
    BT::Optional<double> base_footprint_roll = getInput<double>("target_roll");
    BT::Optional<double> base_footprint_pitch = getInput<double>("target_pitch");
    BT::Optional<double> base_footprint_yaw = getInput<double>("target_yaw");
    BT::Optional<double> pregresp_offset = getInput<double>("pregrasp_offset");

    std::string sensor_deploy_frame_names_dynamic_;

    getInput<std::string>("sensor_deploy_frame_names_dynamic", sensor_deploy_frame_names_dynamic_);

    std::size_t pos_comma = sensor_deploy_frame_names_dynamic_.find(",");

    std::string target_frame = sensor_deploy_frame_names_dynamic_.substr(0, pos_comma);
  
    RCLCPP_INFO(node_->get_logger(), "pregrasp started");

    manipulator_.MoveGripperToPose(pose_from_tf.value(), target_frame, base_footprint_x.value(), base_footprint_y.value(), base_footprint_z.value(), base_footprint_roll.value(), base_footprint_pitch.value(), base_footprint_yaw.value(), pregresp_offset.value());

    RCLCPP_INFO(node_->get_logger(), "pregrasp finished");

    sensor_deploy_frame_names_dynamic_.erase(0, pos_comma + 1);
    
    setOutput<std::string>("sensor_deploy_frame_names_dynamic", sensor_deploy_frame_names_dynamic_);

    int no_of_deploy_sensors_dynamic = std::count(sensor_deploy_frame_names_dynamic_.begin(), sensor_deploy_frame_names_dynamic_.end(), ',');

    int no_of_deploy_sensors_{0};
    getInput<int>("no_of_deploy_sensors", no_of_deploy_sensors_);

    RCLCPP_INFO(node_->get_logger(), "%d sensors already deployed!", no_of_deploy_sensors_ - no_of_deploy_sensors_dynamic); 
    RCLCPP_INFO(node_->get_logger(), "%d sensors yet to be deployed!", no_of_deploy_sensors_dynamic);

    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPregrasp::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorPregrasp::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorPregrasp::providedPorts()
{
    return {BT::BidirectionalPort<std::string>("sensor_deploy_frame_names_dynamic"), BT::InputPort<int>("no_of_deploy_sensors"), BT::InputPort<bool>("pose_from_tf"), BT::InputPort<double>("target_x"), BT::InputPort<double>("target_y"), BT::InputPort<double>("target_z"), BT::InputPort<double>("pregrasp_offset"), BT::InputPort<double>("target_roll"), BT::InputPort<double>("target_pitch"), BT::InputPort<double>("target_yaw")};
}

#pragma endregion

#pragma region ManipulatorPostgraspRetreat

/**
 * @brief Construct a new Manipulator Postgrasp Retreat:: Manipulator Postgrasp Retreat object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */
ManipulatorPostgraspRetreat::ManipulatorPostgraspRetreat(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
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
BT::NodeStatus ManipulatorPostgraspRetreat::onStart()
{
    LOG_MANI_START(this->name());
    RCLCPP_INFO(node_->get_logger(), "post grasp started");
    manipulator_.MoveLinearVec(0, 0, 0.12);
    RCLCPP_INFO(node_->get_logger(), "post grasp finished");
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPostgraspRetreat::onRunning()
{   
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorPostgraspRetreat::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorPostgraspRetreat::providedPorts()
{
    RCLCPP_WARN(rclcpp::get_logger("ManipulatorPostgraspRetreat"), "returning empty BT::PortsList!");
    return {};
}

#pragma endregion

#pragma region ManipulatorDrop

/**
 * @brief Construct a new Manipulator Drop :: Manipulator Drop an object
 * 
 * @param name The name of the behavior
 */
ManipulatorDrop::ManipulatorDrop(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
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
BT::NodeStatus ManipulatorDrop::onStart()
{
    LOG_MANI_START(this->name());
    RCLCPP_INFO(node_->get_logger(), "going to drop object");
    manipulator_.DropObject();
    RCLCPP_INFO(node_->get_logger(), "object dropped");
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorDrop::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorDrop::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorDrop::providedPorts()
{
    RCLCPP_WARN(rclcpp::get_logger("ManipulatorDrop"), "returning empty BT::PortsList!");
    return {};
}


#pragma endregion

#pragma region ManipulatorScanPose

/**
 * @brief Construct a new Manipulator Scan Pose:: Manipulator Scan Pose object
 * 
 * @param name The name of the behavior
 */
ManipulatorScanPose::ManipulatorScanPose(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
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
BT::NodeStatus ManipulatorScanPose::onStart()
{
    LOG_MANI_START(this->name());
    RCLCPP_INFO(node_->get_logger(), "moving EE to scan position");
    manipulator_.MoveToScanningPosition();
    RCLCPP_INFO(node_->get_logger(), "moved EE to scan position");
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorScanPose::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorScanPose::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorScanPose::providedPorts()
{
    RCLCPP_WARN(rclcpp::get_logger("ManipulatorScanPose"), "returning empty BT::PortsList!");
    return {};
}

#pragma endregion
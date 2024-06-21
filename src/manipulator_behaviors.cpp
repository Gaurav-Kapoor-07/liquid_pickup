#include "manipulator_behaviors.h"

#pragma region ManipulatorGraspTomato

/**
 * @brief Construct a new Manipulator Grasp Tomato:: Manipulator Grasp Tomato object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */

ManipulatorGraspTomato::ManipulatorGraspTomato(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
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
BT::NodeStatus ManipulatorGraspTomato::onStart()
{
    LOG_MANI_START(this->name());

    BT::Optional<std::string> tomato_action = getInput<std::string>("action");
    BT::Optional<double> tomato_base_footprint_x = getInput<double>("target_x");
    BT::Optional<double> tomato_base_footprint_y = getInput<double>("target_y");
    BT::Optional<double> tomato_base_footprint_z = getInput<double>("target_z");
    BT::Optional<double> tomato_base_footprint_roll = getInput<double>("target_roll");
    BT::Optional<double> tomato_base_footprint_pitch = getInput<double>("target_pitch");
    BT::Optional<double> tomato_base_footprint_yaw = getInput<double>("target_yaw");

    RCLCPP_INFO(node_->get_logger(), "moving gripper to tomato");
    manipulator_.MoveGripperToTomato(tomato_action.value(), tomato_base_footprint_x.value(), tomato_base_footprint_y.value(), tomato_base_footprint_z.value(), tomato_base_footprint_roll.value(), tomato_base_footprint_pitch.value(), tomato_base_footprint_yaw.value());
    RCLCPP_INFO(node_->get_logger(), "moved gripper to tomato");
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorGraspTomato::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorGraspTomato::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorGraspTomato::providedPorts()
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

    BT::Optional<std::string> tomato_action = getInput<std::string>("action");
    BT::Optional<double> tomato_base_footprint_x = getInput<double>("target_x");
    BT::Optional<double> tomato_base_footprint_y = getInput<double>("target_y");
    BT::Optional<double> tomato_base_footprint_z = getInput<double>("target_z");
    BT::Optional<double> tomato_base_footprint_roll = getInput<double>("target_roll");
    BT::Optional<double> tomato_base_footprint_pitch = getInput<double>("target_pitch");
    BT::Optional<double> tomato_base_footprint_yaw = getInput<double>("target_yaw");
    BT::Optional<double> pregresp_offset = getInput<double>("pregrasp_offset");
  
    RCLCPP_INFO(node_->get_logger(), "pregrasp started");
    manipulator_.MoveGripperToPregraspPose(tomato_action.value(), tomato_base_footprint_x.value(), tomato_base_footprint_y.value(), tomato_base_footprint_z.value(), tomato_base_footprint_roll.value(), tomato_base_footprint_pitch.value(), tomato_base_footprint_yaw.value(), pregresp_offset.value());
    RCLCPP_INFO(node_->get_logger(), "pregrasp finished");
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
    return {BT::InputPort<std::string>("action"), BT::InputPort<double>("target_x"), BT::InputPort<double>("target_y"), BT::InputPort<double>("target_z"), BT::InputPort<double>("pregrasp_offset"), BT::InputPort<double>("target_roll"), BT::InputPort<double>("target_pitch"), BT::InputPort<double>("target_yaw")};
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

#pragma region ManipulatorDropTomato

/**
 * @brief Construct a new Manipulator Drop Tomato:: Manipulator Drop Tomato object
 * 
 * @param name The name of the behavior
 */
ManipulatorDropTomato::ManipulatorDropTomato(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
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
BT::NodeStatus ManipulatorDropTomato::onStart()
{
    LOG_MANI_START(this->name());
    RCLCPP_INFO(node_->get_logger(), "going to drop tomato in basket");
    manipulator_.DropTomatoInBasket();
    RCLCPP_INFO(node_->get_logger(), "tomato dropped");
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorDropTomato::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorDropTomato::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorDropTomato::providedPorts()
{
    RCLCPP_WARN(rclcpp::get_logger("ManipulatorDropTomato"), "returning empty BT::PortsList!");
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
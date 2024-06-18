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

    client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node_, "/summit/move_action");
    // client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node_, "/move_action");

    client_->wait_for_action_server();

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
    BT::Optional<float> tomato_map_x = getInput<float>("target_x");
    BT::Optional<float> tomato_map_y = getInput<float>("target_y");
    BT::Optional<float> tomato_map_z = getInput<float>("target_z");

    if (!tomato_map_x || !tomato_map_y || !tomato_map_z)
    {
        RCLCPP_ERROR(node_->get_logger(), "GOT NO POSE!");

        return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::Point armlink_loc = ba_helper::GetCurrentArmbaseLocation();
    float phi = atan2(tomato_map_y.value() - armlink_loc.y, tomato_map_x.value() - armlink_loc.x);
    geometry_msgs::msg::PoseStamped tomato;
    tomato.header.frame_id = MAP_FRAME;
    tomato.pose.position.x = tomato_map_x.value() - sin(phi) * 0;
    tomato.pose.position.y = tomato_map_y.value() - cos(phi) * 0;
    tomato.pose.position.z = tomato_map_z.value();

    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0, 0, 0);
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);
    tomato.pose.orientation = msg_quat;

    manipulator_.MoveGripperToTomato(tomato);
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorGraspTomato::onRunning()
{
    BT::NodeStatus state = manipulator_.GetNodeStatus(this->name().c_str());
    if (state != BT::NodeStatus::RUNNING)
    {   
        LOG_MANI_STOP(this->name());
    }
    return state;

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
    return {BT::InputPort<float>("target_x"), BT::InputPort<float>("target_y"), BT::InputPort<float>("target_z")};
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

    BT::Optional<float> tomato_map_x = getInput<float>("target_x");
    BT::Optional<float> tomato_map_y = getInput<float>("target_y");
    BT::Optional<float> tomato_map_z = getInput<float>("target_z");

    BT::Optional<float> pregresp_offset = getInput<float>("pregrasp_offset");

    if (!tomato_map_x || !tomato_map_y || !tomato_map_z)
    {
        RCLCPP_ERROR(node_->get_logger(), "GOT NO POSE!");
        return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PoseStamped tomato;
    tomato.header.frame_id = MAP_FRAME;
    geometry_msgs::msg::Point armlink_loc = ba_helper::GetCurrentArmbaseLocation();
    float phi = atan2(tomato_map_y.value() - armlink_loc.y, tomato_map_x.value() - armlink_loc.x);
    tomato.pose.position.x = tomato_map_x.value() - sin(phi) * 0;
    tomato.pose.position.y = tomato_map_y.value() - cos(phi) * 0;
    tomato.pose.position.z = tomato_map_z.value();

    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0, 0, 0);
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);
    tomato.pose.orientation = msg_quat;
  
    manipulator_.MoveGripperToPregraspPose(tomato, pregresp_offset.value());

    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPregrasp::onRunning()
{
    BT::NodeStatus state = manipulator_.GetNodeStatus(this->name().c_str());
    if (state != BT::NodeStatus::RUNNING)
    {   
        LOG_MANI_STOP(this->name());
    }
    return state;
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
    return {BT::InputPort<float>("target_x"),
            BT::InputPort<float>("target_y"),
            BT::InputPort<float>("target_z"),
            BT::InputPort<float>("pregrasp_offset")};
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
    manipulator_.MoveLinearVec(0, 0, 0.12);
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPostgraspRetreat::onRunning()
{   
    BT::NodeStatus state = manipulator_.GetNodeStatus(this->name().c_str());
    if (state != BT::NodeStatus::RUNNING)
    {   
        LOG_MANI_STOP(this->name());
    }
    return state;
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
    return {BT::InputPort<float>("target_x"),
            BT::InputPort<float>("target_y"),
            BT::InputPort<float>("target_z")};
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
    manipulator_.DropTomatoInBasket();
    RCLCPP_INFO(node_->get_logger(), "going to drop tomato in basket");
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorDropTomato::onRunning()
{
    BT::NodeStatus state = manipulator_.GetNodeStatus(this->name().c_str());
    if (state == BT::NodeStatus::SUCCESS)
    {   
        // tomato_queue_->SetTomatoAsPicked();
        // tomato_queue_->AddTomatoToBasketQueue();
    }
    if(state != BT::NodeStatus::RUNNING){
        LOG_MANI_STOP(this->name());
    }
    return state;
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
    manipulator_.MoveToScanningPosition();
    RCLCPP_INFO(node_->get_logger(), "moving EE to scan position");
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorScanPose::onRunning()
{
    BT::NodeStatus state = manipulator_.GetNodeStatus(this->name().c_str());
    if (state != BT::NodeStatus::RUNNING)
    {   
        LOG_MANI_STOP(this->name());
    }
    return state;
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
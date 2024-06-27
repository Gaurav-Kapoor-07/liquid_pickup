#include "navigation_behaviors.h"

#pragma region GoToPose

#pragma region public methods
/**
 * @brief Construct a new Go To Pose:: Go To Pose object
 *
 * @param name The name of the behavior
 * @param config The node configuration
 */
GoToPose::GoToPose(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node) : BT::StatefulActionNode(name, config), manipulator_(node)
{
    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s] Node shared pointer was passed!", this->name().c_str());
    }

    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "/summit/navigate_to_pose");

    if (!action_client_->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    return;
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] Initialized!", this->name().c_str());
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus GoToPose::onStart()
{
    LOG_NAV_START(this->name());
    manipulator_.MoveToDrivingPosition();

    double target_x_ = getInput<double>("target_x").value();
    double target_y_ = getInput<double>("target_y").value();
    double target_yaw_ = getInput<double>("target_yaw").value();

    auto nav_msg = nav2_msgs::action::NavigateToPose::Goal(); 
    
    // 2D pose goal
    nav_msg.pose.header.stamp = node_->get_clock()->now();
    nav_msg.pose.header.frame_id = MAP_FRAME;
    nav_msg.pose.pose.position.x = target_x_;
    nav_msg.pose.pose.position.y = target_y_;
    nav_msg.pose.pose.position.z = 0.0;
    nav_msg.pose.pose.orientation.x = 0.0;
    nav_msg.pose.pose.orientation.y = 0.0;
    nav_msg.pose.pose.orientation.z = std::sin(target_yaw_ / 2.0);
    nav_msg.pose.pose.orientation.w = std::cos(target_yaw_ / 2.0);

    RCLCPP_INFO(node_->get_logger(), "Sending goal: header.frame_id: %s, x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f", nav_msg.pose.header.frame_id.c_str(), nav_msg.pose.pose.position.x, nav_msg.pose.pose.position.y, nav_msg.pose.pose.position.z, nav_msg.pose.pose.orientation.x, nav_msg.pose.pose.orientation.y, nav_msg.pose.pose.orientation.z, nav_msg.pose.pose.orientation.w);

    // Ask server to achieve some goal and wait until it's accepted
    auto goal_handle_future = action_client_->async_send_goal(nav_msg);
    if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
    {
    RCLCPP_ERROR(node_->get_logger(), "send goal call failed :(");
    return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus GoToPose::onRunning()
{
    LOG_NAV_STOP(this->name());
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 *
 */
void GoToPose::onHalted(){};

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList GoToPose::providedPorts()
{
    return {BT::InputPort<double>("target_x"), BT::InputPort<double>("target_y"), BT::InputPort<double>("target_yaw")};
}

#pragma endregion

#pragma endregion

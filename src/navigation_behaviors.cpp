#include "navigation_behaviors.h"

#pragma region GoToPose

#pragma region public methods
/**
 * @brief Construct a new Go To Pose:: Go To Pose object
 *
 * @param name The name of the behavior
 * @param config The node configuration
 */
GoToPose::GoToPose(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node, const rclcpp::executors::SingleThreadedExecutor::SharedPtr executor) : BT::StatefulActionNode(name, config), manipulator_(node)
{
    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s] Node shared pointer was passed!", this->name().c_str());
    }

    if (executor != nullptr)
    {
        executor_ = executor;
        RCLCPP_INFO(node_->get_logger(), "[%s] Executor shared pointer was passed!", this->name().c_str());
    }

    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "/summit/navigate_to_pose");

    if (!action_client_->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    return;
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

    BT::Optional<std::string> behavior_tree_ = getInput<std::string>("behavior_tree");
    BT::Optional<bool> pose_from_tf_ = getInput<bool>("pose_from_tf");
    BT::Optional<double> target_x_ = getInput<double>("target_x").value();
    BT::Optional<double> target_y_ = getInput<double>("target_y").value();
    BT::Optional<double> target_yaw_ = getInput<double>("target_yaw").value();

    auto nav_msg = nav2_msgs::action::NavigateToPose::Goal(); 

    if (pose_from_tf_.value())
    {
        RCLCPP_INFO(node_->get_logger(), "receiving 2D pose goal from TF");

        geometry_msgs::msg::TransformStamped map_to_target_frame;

        try {
            map_to_target_frame = tf_buffer_->lookupTransform(
                MAP_FRAME, LIQUID_FRAME,
                tf2::TimePointZero);
        }   catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(node_->get_logger(),
                "Could not transform %s to %s: %s",
                MAP_FRAME, LIQUID_FRAME, ex.what());
        }

        // 2D pose goal
        nav_msg.pose.header = map_to_target_frame.header;
        nav_msg.pose.pose.position.x = map_to_target_frame.transform.translation.x;
        nav_msg.pose.pose.position.y = map_to_target_frame.transform.translation.y;
        nav_msg.pose.pose.position.z = map_to_target_frame.transform.translation.z;
        nav_msg.pose.pose.orientation.x = map_to_target_frame.transform.rotation.x;
        nav_msg.pose.pose.orientation.y = map_to_target_frame.transform.rotation.y;
        nav_msg.pose.pose.orientation.z = map_to_target_frame.transform.rotation.z;
        nav_msg.pose.pose.orientation.w = map_to_target_frame.transform.rotation.w;
    }

    else
    {
        RCLCPP_INFO(node_->get_logger(), "receiving 2D pose goal from XML string");
        // 2D pose goal
        nav_msg.pose.header.stamp = node_->get_clock()->now();
        nav_msg.pose.header.frame_id = MAP_FRAME;
        nav_msg.pose.pose.position.x = target_x_.value();
        nav_msg.pose.pose.position.y = target_y_.value();
        nav_msg.pose.pose.position.z = 0.0;
        nav_msg.pose.pose.orientation.x = 0.0;
        nav_msg.pose.pose.orientation.y = 0.0;
        nav_msg.pose.pose.orientation.z = std::sin(target_yaw_.value() / 2.0);
        nav_msg.pose.pose.orientation.w = std::cos(target_yaw_.value() / 2.0);
    }

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
    std::string path_to_xml = package_share_directory + "/behavior_trees/";
    nav_msg.behavior_tree = path_to_xml + behavior_tree_.value();

    RCLCPP_INFO(node_->get_logger(), "Sending goal: header.frame_id: %s, x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f, behavior_tree: %s", nav_msg.pose.header.frame_id.c_str(), nav_msg.pose.pose.position.x, nav_msg.pose.pose.position.y, nav_msg.pose.pose.position.z, nav_msg.pose.pose.orientation.x, nav_msg.pose.pose.orientation.y, nav_msg.pose.pose.orientation.z, nav_msg.pose.pose.orientation.w, nav_msg.behavior_tree.c_str());

    // Ask server to achieve some goal and wait until it's accepted
    auto goal_handle_future = action_client_->async_send_goal(nav_msg);

    // if (executor_->spin_until_future_complete(goal_handle_future) !=
    // if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
    // rclcpp::FutureReturnCode::SUCCESS)
    // {
    // RCLCPP_ERROR(node_->get_logger(), "send goal call failed :(");
    // return BT::NodeStatus::FAILURE;
    // }

    goal_handle_ = goal_handle_future.get();
    if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
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
    // Wait for the server to be done with the goal
    auto result_future = action_client_->async_get_result(goal_handle_);

    RCLCPP_INFO(node_->get_logger(), "Waiting for result");
    // if (rclcpp::spin_until_future_complete(node_, result_future) !=
    // rclcpp::FutureReturnCode::SUCCESS)
    // {
    // RCLCPP_ERROR(node_->get_logger(), "get result call failed :(");
    // return BT::NodeStatus::FAILURE;
    // }

    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult wrapped_result = result_future.get();

    switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
            return BT::NodeStatus::FAILURE;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
            return BT::NodeStatus::FAILURE;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
            return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "result received");
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
    return {BT::InputPort<std::string>("behavior_tree"), BT::InputPort<bool>("pose_from_tf"), BT::InputPort<double>("target_x"), BT::InputPort<double>("target_y"), BT::InputPort<double>("target_yaw")};
}

#pragma endregion

#pragma endregion

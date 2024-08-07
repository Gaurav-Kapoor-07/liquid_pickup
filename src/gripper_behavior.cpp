#include "gripper_behavior.h"

#pragma region public

/**
 * @brief Construct a new Gripper Actuator:: Gripper Actuator object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */

// GripperActuator::GripperActuator(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node, const rclcpp::executors::SingleThreadedExecutor::SharedPtr executor): BT::SyncActionNode(name, config)
GripperActuator::GripperActuator(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node, const rclcpp::executors::MultiThreadedExecutor::SharedPtr executor): BT::StatefulActionNode(name, config)
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

    action_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(node_, "/summit/robotiq_gripper_controller/gripper_cmd");

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
BT::NodeStatus GripperActuator::onStart()
{
    BT::Optional<double> gripper_position = getInput<double>("position");
    BT::Optional<double> gripper_max_effort = getInput<double>("max_effort");
    RCLCPP_INFO(node_->get_logger(), "Gripping -> value: %f", gripper_position.value());

    auto grippercommand_msg = control_msgs::action::GripperCommand::Goal(); 
    grippercommand_msg.command.position = gripper_position.value();
    grippercommand_msg.command.max_effort = gripper_max_effort.value();

    RCLCPP_INFO(node_->get_logger(), "Sending goal");

    // Ask server to achieve some goal and wait until it's accepted
    auto goal_handle_future = action_client_->async_send_goal(grippercommand_msg);

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
};

/**
 * @brief method invoked by a RUNNING action.
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus GripperActuator::onRunning()
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

    rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::WrappedResult wrapped_result = result_future.get();

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

    double position = wrapped_result.result->position; // The current gripper gap size (in meters)
    RCLCPP_INFO(node_->get_logger(), "current gripper gap size (in meters) = %f", position);
    double effort = wrapped_result.result->effort;    // The current effort exerted (in Newtons)
    RCLCPP_INFO(node_->get_logger(), "current effort exerted (in Newtons) = %f", effort);
    bool stalled = wrapped_result.result->stalled;      // True iff the gripper is exerting max effort and not moving
    RCLCPP_INFO(node_->get_logger(), "stalled (0: False, 1: True)? = %d", stalled);
    bool reached_goal = wrapped_result.result->reached_goal; // True iff the gripper position has reached the commanded setpoint
    if (reached_goal)
    {
        RCLCPP_INFO(node_->get_logger(), "goal reached");
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "goal failed");
        return BT::NodeStatus::FAILURE;
    }
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 *
 */
void GripperActuator::onHalted(){};

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList GripperActuator::providedPorts()
{
    return {BT::InputPort<double>("position"), BT::InputPort<double>("max_effort")};
}

#pragma endregion

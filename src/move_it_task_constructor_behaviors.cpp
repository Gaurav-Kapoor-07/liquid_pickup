#include "move_it_task_constructor_behaviors.h"

#pragma region MoveItTaskConstructor

/**
 * @brief Construct a new MoveItTaskConstructor :: MoveItTaskConstructor object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */

MoveItTaskConstructor::MoveItTaskConstructor(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config)
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
BT::NodeStatus MoveItTaskConstructor::onStart()
{
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus MoveItTaskConstructor::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void MoveItTaskConstructor::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList MoveItTaskConstructor::providedPorts()
{
    return {BT::InputPort<std::string>("action")};
}

#pragma endregion

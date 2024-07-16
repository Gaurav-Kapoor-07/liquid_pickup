#include "gripper_behavior_bt_action_node.hpp"

GripperAction::GripperAction(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosActionNode<GripperCommand>(name, conf, params)
{
    node_ = params.nh.lock();

    if (node_ != nullptr)
    {
        RCLCPP_INFO(node_->get_logger(), "[%s] Node shared pointer was passed!", this->name().c_str());
    }
}

PortsList GripperAction::providedPorts()
{
    return providedBasicPorts({InputPort<double>("position"), InputPort<double>("max_effort")});
}

bool GripperAction::setGoal(RosActionNode::Goal& goal) 
{
    // get "order" from the Input port
    getInput("position", goal.command.position);
    getInput("max_effort", goal.command.max_effort);
    // return true, if we were able to set the goal correctly.
    return true;
}

NodeStatus GripperAction::onResultReceived(const WrappedResult& wr)
{    
    RCLCPP_INFO(node_->get_logger(), "result received");
    
    double position = wr.result->position; // The current gripper gap size (in meters)
    RCLCPP_INFO(node_->get_logger(), "current gripper gap size (in meters) = %f", position);
    double effort = wr.result->effort;    // The current effort exerted (in Newtons)
    RCLCPP_INFO(node_->get_logger(), "current effort exerted (in Newtons) = %f", effort);
    bool stalled = wr.result->stalled;      // True iff the gripper is exerting max effort and not moving
    RCLCPP_INFO(node_->get_logger(), "stalled (0: False, 1: True)? = %d", stalled);

    bool reached_goal = wr.result->reached_goal; // True iff the gripper position has reached the commanded setpoint
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

NodeStatus GripperAction::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
}
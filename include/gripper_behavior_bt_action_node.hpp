#include <behaviortree_ros2/bt_action_node.hpp>
#include "control_msgs/action/gripper_command.hpp"

// let's define these, for brevity
using GripperCommand = control_msgs::action::GripperCommand;

using namespace BT;

class GripperAction: public RosActionNode<GripperCommand>
{
public:
  GripperAction(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static PortsList providedPorts();

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(RosActionNode::Goal& goal);
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override;

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override;

private:
  rclcpp::Node::SharedPtr node_;
};
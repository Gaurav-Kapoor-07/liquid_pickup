#pragma region includes

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

#include "string"

#pragma endregion

#pragma region SensorsDeploy

/**
 * @brief Receive sensor deploy poses at port
 *
 */
class SensorsDeploy : public BT::StatefulActionNode
{
public:
  SensorsDeploy(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
};

#pragma endregion

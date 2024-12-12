#ifndef GRIPPER_BEHAVIOR_H
#define GRIPPER_BEHAVIOR_H

#pragma region includes

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "ba_interfaces.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "manipulator.h"

#include <climits>
#include <cstdlib>

#pragma endregion

#pragma region GripperActuator

/**
 * @brief Class to control the gripper (open/close) and be sure that the object is attached
 * 
 */
class GripperActuator : public BT::StatefulActionNode
{
public:
    // GripperActuator(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node, const rclcpp::executors::SingleThreadedExecutor::SharedPtr executor);
    GripperActuator(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node, const rclcpp::executors::MultiThreadedExecutor::SharedPtr executor);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();
   
private:
    rclcpp::Node::SharedPtr node_;
    // rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr action_client_;
    rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::SharedPtr goal_handle_;
    Manipulator manipulator_;
    std::string action_name_;
};

#pragma endregion

#endif
#ifndef BA_MOVEITTASKCONSTRUCTOR_BEHAVIOR_H
#define BA_MOVEITTASKCONSTRUCTOR_BEHAVIOR_H

#pragma region includes

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

// #include "manipulator.h"
#include <math.h>
#include "ba_interfaces.h"

#include "rclcpp/rclcpp.hpp"

#pragma endregion

#pragma region MoveItTaskConstructor

/**
 * @brief Class/Behavior which implements the MoveIt Task Constructor
 * 
 */
class MoveItTaskConstructor : public BT::StatefulActionNode
{
public:
    MoveItTaskConstructor(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    // Manipulator manipulator_;
};

#pragma endregion

#endif
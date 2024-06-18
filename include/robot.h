#pragma region includes

#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include "ba_interfaces.h"

#include "behaviortree_cpp/behavior_tree.h"
#include "manipulator.h"

#pragma endregion

// typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> MoveGroupClient;

#pragma region RobotInitializer

/**
 * @brief Class/Behavior to initialize the robot in the beginning
 *
 */
// class RobotInitializer : public BT::SyncActionNode, public IBAInitManipulatorNode, public IBAInitNodeHandle
class RobotInitializer : public BT::SyncActionNode
{
public:
    RobotInitializer(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node);
    // void init(std::shared_ptr<rclcpp::Node> node_handle) override;
    // void init(Manipulator manipulator) override;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    Manipulator manipulator_;
    moveit::core::MoveItErrorCode SetInitialPosition();
    // void LaunchBasket();
};

#pragma endregion
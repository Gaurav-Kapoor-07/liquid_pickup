#pragma region includes

#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include "ba_interfaces.h"

#include "behaviortree_cpp/behavior_tree.h"
#include "manipulator.h"

#pragma endregion

#define CRITICAL_BATTERY_LEVEL 40
// typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> MoveGroupClient;

#pragma region RobotInitializer

/**
 * @brief Class/Behavior to initialize the robot in the beginning
 *
 */
class RobotInitializer : public BT::SyncActionNode, public IBAInitManipulatorNode, public IBAInitNodeHandle
{
public:
    RobotInitializer(const std::string &name, const BT::NodeConfiguration &config);
    // void init(std::shared_ptr<rclcpp::Node> node_handle) override;
    void init(Manipulator manipulator) override;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    Manipulator manipulator_;
    // std::shared_ptr<rclcpp::Node> node_handle_;
    moveit::core::MoveItErrorCode SetInitialPosition();
    // void LaunchBasket();
};

#pragma endregion

#pragma region BatteryCheck

/**
 * @brief Behavior to check the battery of the robot
 *
 */
class BatteryCheck : public BT::ConditionNode, public IBAInitNodeHandle
{
public:
    BatteryCheck(const std::string &name, const BT::NodeConfiguration &config);
    // void init(std::shared_ptr<rclcpp::Node> node_handle) override;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    // std::shared_ptr<rclcpp::Node> node_handle_;
    static rclcpp::Node::SharedPtr node_handle_;
    rclcpp::TimerBase::SharedPtr timer_;
    // ros::Timer timer_;
    bool battery_empty_{false};
    double timer_duration_{1.0};
    // int timer_duration_;
    double start_{0.0};

    void TimerCallback();
};

#pragma endregion

#pragma region BatteryCharge

/**
 * @brief Behavior to charge the battery of the robot
 *
 */
class BatteryCharge : public BT::ConditionNode
{
public:
    BatteryCharge(const std::string &name, const BT::NodeConfiguration &config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    // std::shared_ptr<rclcpp::Node> node_handle_2;
};

#pragma endregion
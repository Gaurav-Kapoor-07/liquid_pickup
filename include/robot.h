#pragma region includes

#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include "ba_interfaces.h"

#include "behaviortree_cpp/behavior_tree.h"
#include "manipulator.h"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "tinyxml.h"

#pragma endregion

#pragma region RobotInitializer

/**
 * @brief Class/Behavior to initialize the robot in the beginning
 *
 */
class RobotInitializer : public BT::SyncActionNode
{
public:
    RobotInitializer(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    Manipulator manipulator_;
    moveit::core::MoveItErrorCode SetInitialPosition();
    void LaunchSwabContainer();
    void LaunchSwab();
};

#pragma endregion
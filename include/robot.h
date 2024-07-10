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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "tinyxml.h"
#include "vector"

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
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    unsigned int no_of_deploy_sensors{0};
    moveit::core::MoveItErrorCode SetInitialPosition();
    void LaunchSwabContainer();
    void LaunchSwab();
    void getDeploySensorPoses();
};

#pragma endregion
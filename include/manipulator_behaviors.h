#ifndef BA_MANIPULATOR_BEHAVIOR_H
#define BA_MANIPULATOR_BEHAVIOR_H

#pragma region includes

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "manipulator.h"
#include <math.h>
#include "ba_interfaces.h"

#include "rclcpp/rclcpp.hpp"

#include "moveit/utils/moveit_error_code.h"

#include "std_msgs/msg/bool.hpp"

#include "time_logger.h"
#ifdef TIME_LOGGER_ON
#define LOG_MANI_START(val) BATimeLogger::LogMoveGroup(val, log_start)
#define LOG_MANI_STOP(val) BATimeLogger::LogMoveGroup(val, log_stop)
#else
#define LOG_MANI_START(val)
#define LOG_MANI_STOP(val)
#endif

#pragma endregion

using std::placeholders::_1;

#define UR5_WORKING_RADIUS 0.95

#pragma region ManipulatorGrasp

/**
 * @brief Class/Behavior which grasps an object
 * 
 */
class ManipulatorGrasp : public BT::StatefulActionNode
{
public:
    ManipulatorGrasp(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    Manipulator manipulator_;
};

#pragma endregion

#pragma region ManipulatorPregraspPlan

/**
 * @brief Class/Behavior to go into the pregrasp pose
 * 
 */
class ManipulatorPregraspPlan : public BT::StatefulActionNode
{
public:
    ManipulatorPregraspPlan(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node);
    void topic_callback(const std_msgs::msg::Bool::SharedPtr msg);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    Manipulator manipulator_;
    int count_{0};
    bool flag_{false};
    bool trajectory_execute_{false};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trajectory_execute_subscription_;
    moveit_msgs::msg::RobotTrajectory plan_trajectory_;
};

#pragma endregion

#pragma region ManipulatorPregraspExecute

/**
 * @brief Class/Behavior to go into the pregrasp pose
 * 
 */
class ManipulatorPregraspExecute : public BT::StatefulActionNode
{
public:
    ManipulatorPregraspExecute(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    Manipulator manipulator_;
    std::string error_message_;
};

#pragma endregion

#pragma region ManipulatorPostgraspRetreat

/**
 * @brief Class/Behavior to retreat the robot of the arm after grasping
 * 
 */
class ManipulatorPostgraspRetreat : public BT::StatefulActionNode
{
public:
    ManipulatorPostgraspRetreat(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    Manipulator manipulator_;
};

#pragma endregion

#pragma region ManipulatorDrop

/**
 * @brief Class/Behavior to drop the object into the attached swab container after grasping
 * 
 */
class ManipulatorDrop : public BT::StatefulActionNode
{
public:
    ManipulatorDrop(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    Manipulator manipulator_;
};

#pragma endregion

#pragma region ManipulatorScanPose

/**
 * @brief Class/Behavior to move the robots arm into the scanning position
 * 
 */
class ManipulatorScanPose : public BT::StatefulActionNode
{
public:
    ManipulatorScanPose(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();
    
private:
    rclcpp::Node::SharedPtr node_;
    Manipulator manipulator_;
};

#pragma endregion

#endif
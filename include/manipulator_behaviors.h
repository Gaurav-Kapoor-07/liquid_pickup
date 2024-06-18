#ifndef BA_MANIPULATOR_BEHAVIOR_H
#define BA_MANIPULATOR_BEHAVIOR_H

#pragma region includes

#include "behaviortree_cpp/behavior_tree.h"
// #include <optional>
#include "behaviortree_cpp/bt_factory.h"

#include "manipulator.h"
// #include "ros_logs.h"
// #include "tomato_queue.h"
#include <math.h>
#include "ba_interfaces.h"
#include "std_msgs/msg/float64.hpp"
#include "helper.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <moveit_msgs/action/move_group.hpp>

#include "time_logger.h"
#ifdef TIME_LOGGER_ON
#define LOG_MANI_START(val) BATimeLogger::LogMoveGroup(val, log_start)
#define LOG_MANI_STOP(val) BATimeLogger::LogMoveGroup(val, log_stop)
#else
#define LOG_MANI_START(val)
#define LOG_MANI_STOP(val)
#endif

#pragma endregion

#define UR5_WORKING_RADIUS 0.95

// typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> MoveGroupClient;

// #pragma region DequeueTomato

/**
 * @brief Gets location from a queue of locations read from a list
 * 
 */
// class DequeueTomato : public BT::SyncActionNode, public IBAInitTomatoQueue
// {
// public:
//     DequeueTomato(const std::string &name, const BT::NodeConfiguration &config);
//     void init(TomatoQueue &tqueue) override;
//     BT::NodeStatus tick() override;
//     static BT::PortsList providedPorts();

// private:
//     TomatoQueue *tomato_queue_;
//     std::list<TomatoCoordinates> reachable_tomatoes_;
//     geometry_msgs::msg::PoseStamped TransformTomatoToPose(TomatoCoordinates tomato, std::string frame);
// };

// #pragma endregion

// #pragma region FilterTomatoQueue

/**
 * @brief Filters the tomato queue to only keep the reachable ones
 * 
 */
// class FilterTomatoQueue : public BT::SyncActionNode, public IBAInitTomatoQueue
// {
// public:
//     FilterTomatoQueue(const std::string &name, const BT::NodeConfiguration &config);
//     void init(TomatoQueue &tomato_queue) override;
//     BT::NodeStatus tick() override;
//     static BT::PortsList providedPorts();

// private:
//     TomatoQueue *tomato_queue_;
// };

// #pragma endregion

#pragma region ManipulatorGraspTomato

/**
 * @brief Class/Behavior which grasps a tomato
 * 
 */
class ManipulatorGraspTomato : public BT::StatefulActionNode
{
public:
    ManipulatorGraspTomato(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr client_;
    Manipulator manipulator_;
};

#pragma endregion

#pragma region ManipulatorPregrasp

/**
 * @brief Class/Behavior to go into the pregrasp pose
 * 
 */
class ManipulatorPregrasp : public BT::StatefulActionNode
{
public:
    ManipulatorPregrasp(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    Manipulator manipulator_;
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

#pragma region ManipulatorDropTomato

/**
 * @brief Class/Behavior to drop the tomato into the attached basket after grasping
 * 
 */
class ManipulatorDropTomato : public BT::StatefulActionNode
{
public:
    ManipulatorDropTomato(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node);
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
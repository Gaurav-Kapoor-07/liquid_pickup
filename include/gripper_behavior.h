#ifndef GRIPPER_BEHAVIOR_H
#define GRIPPER_BEHAVIOR_H

#pragma region includes

#include "behaviortree_cpp/behavior_tree.h"
#include "std_msgs/msg/float64.hpp"
#include "ros_logs.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

#pragma endregion

#pragma region defines

#define DEFAULT_CONTACT_TOPIC   "gazebo/default/summit_xl/contacts"
#define GRIPPER_TOPIC           "gripper_left_controller/command"

#pragma endregion

#pragma region GripperActuator

/**
 * @brief Class to control the gripper (open/close) and be sure that the object is attached
 * 
 */
class GripperActuator : public BT::SyncActionNode
{
public:
    GripperActuator(const std::string &name, const BT::NodeConfiguration &config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
private:
    rclcpp::Node::SharedPtr node_;
    // ros::Publisher grasp_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr grasp_publisher_;
    // ros::Subscriber grasp_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr grasp_subscriber_;
    bool object_attached_;
    bool object_attached_before_;
    // void ResultCallback(const std_msgs::msg::Float64 &message);
    void ResultCallback(const std_msgs::msg::Float64::SharedPtr message);
};

#pragma endregion

#endif
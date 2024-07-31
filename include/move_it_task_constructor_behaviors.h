#ifndef BA_MOVEITTASKCONSTRUCTOR_BEHAVIOR_H
#define BA_MOVEITTASKCONSTRUCTOR_BEHAVIOR_H

#pragma region includes

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <math.h>
#include "ba_interfaces.h"
#include "ba_frames_summit_xl.h"
#include "manipulator.h"

#include "rclcpp/rclcpp.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#pragma endregion

namespace mtc = moveit::task_constructor;

#pragma region MoveItTaskConstructor

/**
 * @brief Class/Behavior which implements the MoveIt Task Constructor
 * 
 */
class MoveItTaskConstructor : public BT::StatefulActionNode
{
public:
    MoveItTaskConstructor(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node);
    
    // Compose an MTC task from a series of stages.
    mtc::Task createTask();
    
    void doTask();
    
    void setupPlanningScene();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
    // std::shared_ptr<moveit::planning_interface::MoveGroupInterface> manipulator_;
};

#pragma endregion

#endif
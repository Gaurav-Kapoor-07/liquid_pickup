#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

#pragma region includes

#include "rclcpp/rclcpp.hpp"
// #include "ros/message.h"
#include <geometry_msgs/msg/pose.hpp>
// #include "tf/tf.h"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include "helper.h"
#include <stdio.h>
#include "yaml-cpp/yaml.h"
#include <math.h>

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// #include <move_base_msgs/MoveBaseAction.h>

#include "ba_frames_summit_xl.h"

#pragma endregion

#pragma region defines

#define ROBOT_DESCRIPTION               "/summit_xl/robot_description"
#define GROUP_NAME                      "manipulator"

#define MY_PI                           3.14159
#define PREGRASP_DISTANCE               0.35    // distance offset of ee from tomato for validation scan
#define TCP_OFFSET_Z                    0.10    // vertical offset
#define TCP_OFFSET_XY                   0.15    // distance offset in XY-plane
#define MANIPULATOR_JOINT_TOLERANCE     0.01
#define MANIPULATOR_TOLERANCE_SMALL     0.01
#define MANIPULATOR_TOLERANCE_PREGRASP  0.015
#define MANIPULATOR_TOLERANCE_MEDIUM    0.05
#define MANIPULATOR_TOLERANCE_LARGE     0.1

#pragma endregion

#pragma region Manipulator

/**
 * @brief Class which actually controls the manipulator
 * 
 */
class Manipulator 
{
public:
    Manipulator();
    // void init(ros::NodeHandle& node_handle);
    BT::NodeStatus GetNodeStatus(const char* name);
    // moveit::core::MoveItErrorCode MoveGripperToPregraspPose(geometry_msgs::PoseStamped& tomato_pose, float offset);
    // moveit::core::MoveItErrorCode MoveGripperToTomato(geometry_msgs::PoseStamped& tomato_pose);
    double MoveLinear(geometry_msgs::msg::Pose end_pose, bool check_collision = true);
    double MoveLinearVec(float x, float y, float z);
    // moveit::core::MoveItErrorCode DropTomatoInBasket(void);
    // moveit::core::MoveItErrorCode MoveToInitialPosition(void);
    // moveit::core::MoveItErrorCode MoveToDrivingPosition(void);
    // moveit::core::MoveItErrorCode MoveToScanningPosition(void);
private:
    // moveit::planning_interface::MoveGroupInterface *manipulator_;
    geometry_msgs::msg::PoseStamped drop_pose_;
    std::map<std::string, double> initial_position_, driving_position_, scanning_position_;
    // ros::NodeHandle node_handle_;
    void InitializeSummitXlPoses(void);
    void InitializeInitialPose(void);
    void InitializeDrivingPose(void);
    void InitializeScanningPose(void);
    void InitializeDropPose(void);
};

#pragma endregion

#endif

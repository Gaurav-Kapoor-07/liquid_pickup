#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

#pragma region includes

#include "ros/ros.h"
#include "ros/message.h"
#include <geometry_msgs/Pose.h>
#include "tf/tf.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "helper.h"
#include <stdio.h>
#include "yaml-cpp/yaml.h"
#include <math.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

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
    void init(ros::NodeHandle& node_handle);
    BT::NodeStatus GetNodeStatus(const char* name);
    moveit::core::MoveItErrorCode MoveGripperToPregraspPose(geometry_msgs::PoseStamped& tomato_pose, float offset);
    moveit::core::MoveItErrorCode MoveGripperToTomato(geometry_msgs::PoseStamped& tomato_pose);
    double MoveLinear(geometry_msgs::Pose end_pose, bool check_collision = true);
    double MoveLinearVec(float x, float y, float z);
    moveit::core::MoveItErrorCode DropTomatoInBasket(void);
    moveit::core::MoveItErrorCode MoveToInitialPosition(void);
    moveit::core::MoveItErrorCode MoveToDrivingPosition(void);
    moveit::core::MoveItErrorCode MoveToScanningPosition(void);
private:
    moveit::planning_interface::MoveGroupInterface *manipulator_;
    geometry_msgs::PoseStamped drop_pose_;
    std::map<std::string, double> initial_position_, driving_position_, scanning_position_;
    ros::NodeHandle node_handle_;
    void InitializeSummitXlPoses(void);
    void InitializeInitialPose(void);
    void InitializeDrivingPose(void);
    void InitializeScanningPose(void);
    void InitializeDropPose(void);
};

#pragma endregion

#endif

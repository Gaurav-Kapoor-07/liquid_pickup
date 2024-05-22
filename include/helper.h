#ifndef BA_HELPER_H
#define BA_HELPER_H

#pragma region includes

#include <math.h>
// #include <pcl/point_types.h>
#include "rclcpp/rclcpp.hpp"
// #include "ros/message.h"
#include <geometry_msgs/msg/pose.hpp>
// #include "tf/tf.h"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"
#include "ba_frames_summit_xl.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#pragma endregion

namespace ba_helper
{
    float ConvertDegreesToRadians(float degrees);
    float ConvertQuaternionToAngle(geometry_msgs::msg::Quaternion quat);
    float CalculateGripperParameterFromDesiredAirgap(float airgap);
    geometry_msgs::msg::Point GetCurrentArmbaseLocation(void);
    geometry_msgs::msg::PoseStamped GetCurrentPoseInMapFrame(const char frame[],
                                                          float x = 0, float y = 0, float z = 0,
                                                          float roll = 0, float pitch = 0, float yaw = 0);
}

#endif
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
    double ConvertDegreesToRadians(double degrees);
    double ConvertQuaternionToAngle(geometry_msgs::msg::Quaternion quat);
    double CalculateGripperParameterFromDesiredAirgap(double airgap);
    geometry_msgs::msg::Point GetCurrentArmbaseLocation(void);
    geometry_msgs::msg::PoseStamped GetCurrentPoseInMapFrame(const char frame[],
                                                          double x = 0, double y = 0, double z = 0,
                                                          double roll = 0, double pitch = 0, double yaw = 0);
}

#endif
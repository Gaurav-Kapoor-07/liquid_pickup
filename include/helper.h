#ifndef BA_HELPER_H
#define BA_HELPER_H

#pragma region includes

#include <math.h>
#include <pcl/point_types.h>
#include "ros/ros.h"
#include "ros/message.h"
#include <geometry_msgs/Pose.h>
#include "tf/tf.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>
#include "ba_frames_summit_xl.h"

#pragma endregion

namespace ba_helper
{
    float ConvertDegreesToRadians(float degrees);
    float ConvertQuaternionToAngle(geometry_msgs::Quaternion quat);
    float CalculateGripperParameterFromDesiredAirgap(float airgap);
    geometry_msgs::Point GetCurrentArmbaseLocation(void);
    geometry_msgs::PoseStamped GetCurrentPoseInMapFrame(const char frame[],
                                                          float x = 0, float y = 0, float z = 0,
                                                          float roll = 0, float pitch = 0, float yaw = 0);
}

#endif
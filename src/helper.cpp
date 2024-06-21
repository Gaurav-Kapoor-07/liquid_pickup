#include "helper.h"

/**
 * @brief Converts an angle in degrees into radians
 *
 * @param degrees angle to convert in degrees
 * @return double converted angle in radians
 */
double ba_helper::ConvertDegreesToRadians(double degrees)
{
    double radians = degrees / 180 * M_PI;
    return radians;
}

/**
 * @brief Converts a quaternion into an angle, based on the function: Theta = 2*atan2(sqrt(x^2+y^2+z^2), w)
 *
 * @param quat The quaternion to transform
 * @return double The angle in radians
 */
double ba_helper::ConvertQuaternionToAngle(geometry_msgs::msg::Quaternion quat)
{
    double axis = 0;
    axis += quat.x * quat.x;
    axis += quat.y * quat.y;
    axis += quat.z * quat.z;
    axis = std::sqrt(axis);
    return 2 * std::atan2(axis, quat.w);
}

/**
 * @brief linearisation for gripper command --> calculates msg payload to be published to gripper controller from airgap [m] you want the gripper to open
 *
 * @param airgap Desired opening of the gripper in [m]
 * @return double value to publish to gripper controller for danfoa 140 2f robotiq
 */
double ba_helper::CalculateGripperParameterFromDesiredAirgap(double airgap)
{
    double result = -5.40444426 * airgap + 0.71065;
    if (result > 0.7)
    {
        result = 0.7;
    }
    if (result < 0.0)
    {
        result = 0.0;
    }
    return result;
}

/**
 * @brief Gets the current armbase location in the map frame
 *
 * @return geometry_msgs::msg::Point The current armbase location
 */
geometry_msgs::msg::Point ba_helper::GetCurrentArmbaseLocation(void)
{
    geometry_msgs::msg::PoseStamped manipulator_base_map = GetCurrentPoseInMapFrame(ARM_BASE_FRAME);
    return manipulator_base_map.pose.position;
}

/**
 * @brief Calculates the current pose into the map frame and returns it
 *
 * @param source_frame The source frame
 * @param x The x location
 * @param y The y location
 * @param z The z location
 * @param R The roll angle
 * @param P The pith angle
 * @param Y The yaw angle
 * @return geometry_msgs::msg::PoseStamped The current position in the map frame
 */
geometry_msgs::msg::PoseStamped ba_helper::GetCurrentPoseInMapFrame(const char source_frame[],
                                                               double x, double y, double z,
                                                               double roll, double pitch, double yaw)
{
    geometry_msgs::msg::PoseStamped current_position_base_frame;
    geometry_msgs::msg::PoseStamped current_position_map_frame;

    current_position_base_frame.pose.position.x = x;
    current_position_base_frame.pose.position.y = y;
    current_position_base_frame.pose.position.z = z;

    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

    current_position_base_frame.pose.orientation = msg_quat;

    current_position_base_frame.header.frame_id = source_frame;

    current_position_map_frame.header.frame_id = MAP_FRAME;

    return current_position_map_frame;
}

#include "manipulator.h"

#pragma region Manipulator

#pragma region public

/**
 * @brief Construct a new Manipulator:: Manipulator object
 * 
 */
Manipulator::Manipulator(const rclcpp::Node::SharedPtr node)
{
    if (node == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("Manipulator"), "nullptr was passed!");
        return;
    }
       
    node_ = node;

    RCLCPP_INFO(node_->get_logger(), "Node shared pointer was passed!");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    yaml_file = node_->get_parameter("yaml_file").as_string();

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("liquid_pickup");

    std::string path_to_yaml = package_share_directory + "/config/";

    arm_positions = YAML::LoadFile(path_to_yaml + yaml_file);

    Manipulator::InitializeSummitXlPoses();

    moveit::planning_interface::MoveGroupInterface::Options manipulator_options_(GROUP_NAME, ROBOT_DESCRIPTION, "/summit");

    manipulator_ = new moveit::planning_interface::MoveGroupInterface(node_, manipulator_options_);

    manipulator_->allowReplanning(true);
}

/**
 * @brief Moves the gripper to the pregrasp pose in a certain distance off the tomato
 * 
 * @param tomato_pose The endpose to reach
 * @param offset The offset to the end pose
 * @return moveit::core::MoveItErrorCode The errorcode
 */
moveit::core::MoveItErrorCode Manipulator::MoveGripperToPregraspPose(float offset)
{
    manipulator_->setGoalPositionTolerance(MANIPULATOR_TOLERANCE_PREGRASP);

    geometry_msgs::msg::TransformStamped base_footprint_to_tomato_frame;

    try {
        base_footprint_to_tomato_frame = tf_buffer_->lookupTransform(
            BASE_FRAME, LIQUID_FRAME,
            tf2::TimePointZero);
    }   catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(node_->get_logger(),
            "Could not transform %s to %s: %s",
            BASE_FRAME, LIQUID_FRAME, ex.what());
    }

    float y_offset = (base_footprint_to_tomato_frame.transform.translation.y) < 0 ? 0.1 : -0.1;

    geometry_msgs::msg::PoseStamped tomato_base_footprint;

    tomato_base_footprint.header = base_footprint_to_tomato_frame.header;
    tomato_base_footprint.pose.position.x = base_footprint_to_tomato_frame.transform.translation.x;
    tomato_base_footprint.pose.position.y = base_footprint_to_tomato_frame.transform.translation.y;
    tomato_base_footprint.pose.position.z = base_footprint_to_tomato_frame.transform.translation.z;

    tomato_base_footprint.pose.position.z += TCP_OFFSET_Z;

    float angle = atan2(tomato_base_footprint.pose.position.y, tomato_base_footprint.pose.position.x);

    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0, M_PI / 6, angle);
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);
    tomato_base_footprint.pose.orientation = msg_quat;
    tomato_base_footprint.pose.position.x -= cos(angle) * (offset+TCP_OFFSET_XY);
    tomato_base_footprint.pose.position.y -= sin(angle) * (offset+TCP_OFFSET_XY);

    manipulator_->setPoseReferenceFrame(tomato_base_footprint.header.frame_id);
    manipulator_->setPoseTarget(tomato_base_footprint);
    manipulator_->setPlanningTime(5);
    return manipulator_->asyncMove();
}

/**
 * @brief Moves the gripper to the tomato
 * 
 * @param tomato_pose The endpose to reach
 * @return moveit::core::MoveItErrorCode The errorcode
 */
moveit::core::MoveItErrorCode Manipulator::MoveGripperToTomato()
{
    manipulator_->setGoalPositionTolerance(MANIPULATOR_TOLERANCE_SMALL);

    geometry_msgs::msg::TransformStamped base_footprint_to_tomato_frame;

    try {
        base_footprint_to_tomato_frame = tf_buffer_->lookupTransform(
            BASE_FRAME, LIQUID_FRAME,
            tf2::TimePointZero);
    }   catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(node_->get_logger(),
            "Could not transform %s to %s: %s",
            BASE_FRAME, LIQUID_FRAME, ex.what());
    }

    float y_offset = (base_footprint_to_tomato_frame.transform.translation.y) < 0 ? 0.1 : -0.1;
    
    geometry_msgs::msg::PoseStamped tomato_base_footprint;

    tomato_base_footprint.header = base_footprint_to_tomato_frame.header;
    tomato_base_footprint.pose.position.x = base_footprint_to_tomato_frame.transform.translation.x;
    tomato_base_footprint.pose.position.y = base_footprint_to_tomato_frame.transform.translation.y;
    tomato_base_footprint.pose.position.z = base_footprint_to_tomato_frame.transform.translation.z;

    float angle = atan2(tomato_base_footprint.pose.position.y, tomato_base_footprint.pose.position.x);
    tomato_base_footprint.pose.position.x -= cos(angle) * TCP_OFFSET_XY;
    tomato_base_footprint.pose.position.y -= sin(angle) * TCP_OFFSET_XY;
    tomato_base_footprint.pose.position.z += TCP_OFFSET_Z;
    
    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0, M_PI / 6, angle);
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);
    tomato_base_footprint.pose.orientation = msg_quat;

    MoveLinear(tomato_base_footprint.pose, false);
    return moveit::core::MoveItErrorCode::SUCCESS;
}

/**
 * @brief Moves the gripper linearly
 * 
 * @param end_pose The pose to reach
 * @param check_collision Bool whether to check for collisions or not
 * @return double Return a value that is between 0.0 and 1.0 indicating the fraction of the path achieved as described by the waypoints. Return -1.0 in case of error.
 */
double Manipulator::MoveLinear(geometry_msgs::msg::Pose end_pose, bool check_collision)
{
    std::vector<geometry_msgs::msg::Pose> direction;
    direction.push_back(end_pose);
    manipulator_->setPoseReferenceFrame(BASE_FRAME);
    moveit_msgs::msg::RobotTrajectory trajectory;
    double res = manipulator_->computeCartesianPath(direction, 0.01, 10, trajectory, check_collision);
    if (res >= 0)
    {
        manipulator_->execute(trajectory);
    }
    return res;
}

/**
 * @brief Moves the gripper linearly based on three coordinates
 * 
 * @param x The x coordinate
 * @param y The y coordinate
 * @param z The z coordinate
 * @return double Return a value that is between 0.0 and 1.0 indicating the fraction of the path achieved as described by the waypoints. Return -1.0 in case of error.
 */
double Manipulator::MoveLinearVec(float x, float y, float z){
    // geometry_msgs::msg::PoseStamped ee = manipulator_->getPoseTarget();
    
    // workaround for above function
    geometry_msgs::msg::PoseStamped ee;
    ee.header.frame_id = "arm_tool0";
    
    geometry_msgs::msg::TransformStamped transform_ee_base_frame;

    try {
        transform_ee_base_frame = tf_buffer_->lookupTransform(
            BASE_FRAME, ee.header.frame_id,
            tf2::TimePointZero);
    }   catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(node_->get_logger(),
            "Could not transform %s to %s: %s",
            BASE_FRAME, ee.header.frame_id.c_str(), ex.what());
    }

    geometry_msgs::msg::PoseStamped ee_base_frame;

    ee_base_frame.header = transform_ee_base_frame.header;
    ee_base_frame.pose.position.x = transform_ee_base_frame.transform.translation.x;
    ee_base_frame.pose.position.y = transform_ee_base_frame.transform.translation.y;
    ee_base_frame.pose.position.z = transform_ee_base_frame.transform.translation.z;
    ee_base_frame.pose.orientation.x = transform_ee_base_frame.transform.rotation.x;
    ee_base_frame.pose.orientation.y = transform_ee_base_frame.transform.rotation.y;
    ee_base_frame.pose.orientation.z = transform_ee_base_frame.transform.rotation.z;
    ee_base_frame.pose.orientation.w = transform_ee_base_frame.transform.rotation.w;

    ee_base_frame.pose.position.x += x;
    ee_base_frame.pose.position.y += y;
    ee_base_frame.pose.position.z += z;
    double res = MoveLinear(ee_base_frame.pose, false);
    return res;
}

/**
 * @brief Moves the UR5-Robot to its drop position
 * 
 * @return moveit::core::MoveItErrorCode The errorcode
 */
moveit::core::MoveItErrorCode Manipulator::DropTomatoInBasket(void)
{
    manipulator_->setGoalPositionTolerance(MANIPULATOR_TOLERANCE_LARGE);
    manipulator_->setPoseReferenceFrame(BASE_FRAME);
    manipulator_->setPoseTarget(drop_pose_);
    manipulator_->setPlanningTime(30);
    return manipulator_->move();
}

/**
 * @brief Moves the UR5-Robot to its initial position due to the basket.
 * 
 * @return moveit::core::MoveItErrorCode 
 */
moveit::core::MoveItErrorCode Manipulator::MoveToInitialPosition(void) 
{
    manipulator_->setGoalJointTolerance(MANIPULATOR_JOINT_TOLERANCE);
    manipulator_->setPoseReferenceFrame(BASE_FRAME);
    manipulator_->setJointValueTarget(initial_position_);
    manipulator_->setPlanningTime(30);
    return manipulator_->move();
}

/**
 * @brief Moves the UR5-Robot to its driving position.
 * 
 * @return moveit::core::MoveItErrorCode 
 */
moveit::core::MoveItErrorCode Manipulator::MoveToDrivingPosition(void) 
{
    manipulator_->setGoalJointTolerance(MANIPULATOR_JOINT_TOLERANCE);
    manipulator_->setPoseReferenceFrame(BASE_FRAME);
    manipulator_->setJointValueTarget(driving_position_);
    manipulator_->setPlanningTime(30);
    return manipulator_->move();
}

/**
 * @brief Moves the UR5-Robot to its scanning position
 * 
 * @return moveit::core::MoveItErrorCode 
 */
moveit::core::MoveItErrorCode Manipulator::MoveToScanningPosition(void)
{
    manipulator_->setGoalJointTolerance(MANIPULATOR_JOINT_TOLERANCE);
    manipulator_->setPoseReferenceFrame(BASE_FRAME);
    manipulator_->setJointValueTarget(scanning_position_);
    manipulator_->setPlanningTime(30);
    return manipulator_->move();
}

#pragma endregion

#pragma region private

/**
 * @brief Initializes several given poses
 * 
 */
void Manipulator::InitializeSummitXlPoses()
{
    Manipulator::InitializeInitialPose();
    Manipulator::InitializeDrivingPose();
    Manipulator::InitializeScanningPose();
    Manipulator::InitializeDropPose();
}

/**
 * @brief Initializes the initial pose due to a bug (-J argument not working in launch file).
 *        see also https://answers.ros.org/question/242151/how-to-set-initial-pose-to-ur5-in-gazebo/ 
 *        no workaround worked reliable for that case 
 * 
 */
void Manipulator::InitializeInitialPose()
{
    std::vector<float> joint_angles = arm_positions["initial_joint_angles"].as<std::vector<float>>();

    initial_position_["arm_shoulder_pan_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[0]);
    initial_position_["arm_shoulder_lift_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[1]);
    initial_position_["arm_elbow_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[2]);
    initial_position_["arm_wrist_1_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[3]);
    initial_position_["arm_wrist_2_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[4]);
    initial_position_["arm_wrist_3_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[5]);
}

/**
 * @brief Initializes the driving pose
 * 
 */
void Manipulator::InitializeDrivingPose()
{
    std::vector<float> joint_angles = arm_positions["driving_joint_angles"].as<std::vector<float>>();

    driving_position_["arm_shoulder_pan_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[0]);
    driving_position_["arm_shoulder_lift_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[1]);
    driving_position_["arm_elbow_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[2]);
    driving_position_["arm_wrist_1_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[3]);
    driving_position_["arm_wrist_2_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[4]);
    driving_position_["arm_wrist_3_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[5]);
}

/**
 * @brief Initializes the scanning pose
 * 
 */
void Manipulator::InitializeScanningPose()
{
    std::vector<float> joint_angles = arm_positions["scan_position_joint_angles"].as<std::vector<float>>();

    scanning_position_["arm_shoulder_pan_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[0]);
    scanning_position_["arm_shoulder_lift_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[1]);
    scanning_position_["arm_elbow_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[2]);
    scanning_position_["arm_wrist_1_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[3]);
    scanning_position_["arm_wrist_2_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[4]);
    scanning_position_["arm_wrist_3_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[5]);
}

/**
 * @brief initialize drop zone: above basket facing downwards
 *
 */
void Manipulator::InitializeDropPose()
{
    std::vector<float> pose = arm_positions["dropping_position"].as<std::vector<float>>();

    drop_pose_.header.frame_id = BASE_FRAME;

    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(ba_helper::ConvertDegreesToRadians(pose[3]), ba_helper::ConvertDegreesToRadians(pose[4]), ba_helper::ConvertDegreesToRadians(pose[5]));
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

    drop_pose_.pose.orientation = msg_quat;
    drop_pose_.pose.position.x = pose[0];
    drop_pose_.pose.position.y = pose[1];
    drop_pose_.pose.position.z = pose[2];
}

#pragma endregion

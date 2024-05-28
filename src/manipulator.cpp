#include "manipulator.h"

#pragma region Manipulator

#pragma region public

/**
 * @brief Construct a new Manipulator:: Manipulator object
 * 
 */
Manipulator::Manipulator()
{
    Manipulator::InitializeSummitXlPoses();

    // moveit::planning_interface::MoveGroupInterface::Options manipulator_options_(GROUP_NAME, ROBOT_DESCRIPTION, ros::NodeHandle());
    // manipulator_ = new moveit::planning_interface::MoveGroupInterface(manipulator_options_);

    node_ = rclcpp::Node::make_shared("InitializeSummitXlPoses");
    moveit::planning_interface::MoveGroupInterface::Options manipulator_options_(GROUP_NAME, ROBOT_DESCRIPTION);
    moveit::planning_interface::MoveGroupInterface move_group(node_, manipulator_options_);

    manipulator_->allowReplanning(true);
}

/**
 * @brief Set the NodeHandle reference object
 * 
 * @param node_handle Reference to the NodeHandle 
 */
void Manipulator::init(rclcpp::Node::SharedPtr node_handle)
{
    node_handle_ = node_handle;
}

/**
 * @brief Gets the current status of the node
 * 
 * @param name The name of the behavior
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus Manipulator::GetNodeStatus(const char* name)
{
    // actionlib::SimpleClientGoalState state = manipulator_->getMoveGroupClient().getState();

    // rclcpp_action::Client state = manipulator_->getMoveGroupClient().getState();
    // auto future = manipulator_->getMoveGroupClient();
    // auto result = future.get();
    // moveit::planning_interface::MoveGroupInterface *manipulator_;
    
    // std::shared_ptr<moveit::planning_interface::MoveGroupInterface> manipulator_ptr(manipulator_);

    auto& action_client = manipulator_->getMoveGroupClient();

    // action_client.async_get_result();

    // Define a callback function for the action client result
    auto result_callback = [](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::WrappedResult&) {};

    // Create a goal handle, assuming it's nullptr for now
    auto goal_handle = std::shared_ptr<rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>>(nullptr);

    // Call async_get_result() with appropriate arguments
    auto result_future = action_client.async_get_result(goal_handle, result_callback);

    auto result_ = result_future.get();
    auto state = result_.code;

    // if (state == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
    // {
    //     RCLCPP_DEBUG(node_->get_logger(), "[%s] reached Pose", name);
    //     return BT::NodeStatus::SUCCESS;
    // }
    // else if (state == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
    //          state == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
    // {
    //     RCLCPP_DEBUG(node_->get_logger(), "[%s] moving to Pose", name);
    //     return BT::NodeStatus::RUNNING;
    // }
    // else
    // {
    //     RCLCPP_ERROR(node_->get_logger(), "[%s] Failed to reach Pose!", name);
    //     return BT::NodeStatus::FAILURE; // TBD --> ignore non-reachable poses just for now
    // }
}

/**
 * @brief Moves the gripper to the pregrasp pose in a certain distance off the tomato
 * 
 * @param tomato_pose The endpose to reach
 * @param offset The offset to the end pose
 * @return moveit::core::MoveItErrorCode The errorcode
 */
moveit::core::MoveItErrorCode Manipulator::MoveGripperToPregraspPose(geometry_msgs::msg::PoseStamped &tomato_pose, float offset)
{
    manipulator_->setGoalPositionTolerance(MANIPULATOR_TOLERANCE_PREGRASP);
    // tf::TransformListener listener;
    // tf2_ros::TransformListener listener;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    geometry_msgs::msg::TransformStamped t;

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    try {
        t = tf_buffer_->lookupTransform(
            BASE_FRAME, tomato_pose.header.frame_id,
            tf2::TimePointZero);
    }   catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(node_->get_logger(),
            "Could not transform %s to %s: %s",
            BASE_FRAME, tomato_pose.header.frame_id.c_str(), ex.what());
    }

    // listener.waitForTransform(BASE_FRAME, tomato_pose.header.frame_id, ros::Time(0), ros::Duration(3.0));

    // auto node = std::make_shared<rclcpp::Node>("MoveGripperToPregraspPose");
    moveit::planning_interface::MoveGroupInterface::Options manipulator_options_(GROUP_NAME, ROBOT_DESCRIPTION);
    moveit::planning_interface::MoveGroupInterface move_group(node_, manipulator_options_);

    float y_offset = (tomato_pose.pose.position.y) < 0 ? 0.1 : -0.1;
    geometry_msgs::msg::PoseStamped tomato_base_footprint;
    // listener.transformPose(BASE_FRAME, tomato_pose, tomato_base_footprint);
    tf2::doTransform(tomato_pose, tomato_base_footprint, tf_buffer_->lookupTransform(
            BASE_FRAME, tomato_pose.header.frame_id,
            tf2::TimePointZero));
    
    tomato_base_footprint.pose.position.z += TCP_OFFSET_Z;
    float angle = atan2(tomato_base_footprint.pose.position.y, tomato_base_footprint.pose.position.x);
    // tomato_base_footprint.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 6, angle);
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
moveit::core::MoveItErrorCode Manipulator::MoveGripperToTomato(geometry_msgs::msg::PoseStamped &tomato_pose)
{
    manipulator_->setGoalPositionTolerance(MANIPULATOR_TOLERANCE_SMALL);
    // tf::TransformListener listener;
    // listener.waitForTransform(BASE_FRAME, tomato_pose.header.frame_id, ros::Time(0), ros::Duration(3.0));

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    geometry_msgs::msg::TransformStamped t;

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    try {
        t = tf_buffer_->lookupTransform(
            BASE_FRAME, tomato_pose.header.frame_id,
            tf2::TimePointZero);
    }   catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(node_->get_logger(),
            "Could not transform %s to %s: %s",
            BASE_FRAME, tomato_pose.header.frame_id.c_str(), ex.what());
    }

    // moveit::planning_interface::MoveGroupInterface::Options manipulator_options_(GROUP_NAME, ROBOT_DESCRIPTION, ros::NodeHandle());

    // auto node = std::make_shared<rclcpp::Node>("MoveGripperToTomato");
    moveit::planning_interface::MoveGroupInterface::Options manipulator_options_(GROUP_NAME, ROBOT_DESCRIPTION);
    moveit::planning_interface::MoveGroupInterface move_group(node_, manipulator_options_);

    float y_offset = (tomato_pose.pose.position.y) < 0 ? 0.1 : -0.1;
    geometry_msgs::msg::PoseStamped tomato_base_footprint;
    // listener.transformPose(BASE_FRAME, tomato_pose, tomato_base_footprint);

    tf2::doTransform(tomato_pose, tomato_base_footprint, tf_buffer_->lookupTransform(
            BASE_FRAME, tomato_pose.header.frame_id,
            tf2::TimePointZero));

    float angle = atan2(tomato_base_footprint.pose.position.y, tomato_base_footprint.pose.position.x);
    tomato_base_footprint.pose.position.x -= cos(angle) * TCP_OFFSET_XY;
    tomato_base_footprint.pose.position.y -= sin(angle) * TCP_OFFSET_XY;
    tomato_base_footprint.pose.position.z += TCP_OFFSET_Z;
    // tomato_base_footprint.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 6, angle);
    
    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0, M_PI / 6, angle);
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

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
    geometry_msgs::msg::PoseStamped ee = manipulator_->getPoseTarget();
    // tf::TransformListener listener;
    // listener.waitForTransform(BASE_FRAME, ee.header.frame_id,ros::Time(0), ros::Duration(3));

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    geometry_msgs::msg::TransformStamped t;

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    try {
        t = tf_buffer_->lookupTransform(
            BASE_FRAME, ee.header.frame_id,
            tf2::TimePointZero);
    }   catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(node_->get_logger(),
            "Could not transform %s to %s: %s",
            BASE_FRAME, ee.header.frame_id.c_str(), ex.what());
    }

    geometry_msgs::msg::PoseStamped ee_base_frame;
    ee_base_frame.header.frame_id = BASE_FRAME;
    // listener.transformPose(BASE_FRAME, ee,ee_base_frame);
    
    tf2::doTransform(ee, ee_base_frame, tf_buffer_->lookupTransform(
            BASE_FRAME, ee.header.frame_id,
            tf2::TimePointZero));

    ee_base_frame.pose.position.x += x;
    ee_base_frame.pose.position.y += y;
    ee_base_frame.pose.position.z += z;
    double res = MoveLinear(ee_base_frame.pose, false);
    return res;
    return 0;
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
    std::string yaml_file;
    // ros::param::get("arm_positions", yaml_file);

    node_->get_parameter("arm_positions", yaml_file);

    // rclcpp::Node::get_parameter("arm_positions", yaml_file);
    YAML::Node arm_positions = YAML::LoadFile(yaml_file);
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
    std::string yaml_file;
    //     ros::param::get("arm_positions", yaml_file);

    // auto node = std::make_shared<rclcpp::Node>("InitializeDrivingPose");
    node_->get_parameter("arm_positions", yaml_file);

    YAML::Node arm_positions = YAML::LoadFile(yaml_file);
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
    std::string yaml_file;
//     ros::param::get("arm_positions", yaml_file);

    // auto node = std::make_shared<rclcpp::Node>("InitializeScanningPose");
    node_->get_parameter("arm_positions", yaml_file);

    YAML::Node arm_positions = YAML::LoadFile(yaml_file);
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
    std::string yaml_file;
//     ros::param::get("arm_positions", yaml_file);
    // auto node = std::make_shared<rclcpp::Node>("InitializeDropPose");
    node_->get_parameter("arm_positions", yaml_file);

    YAML::Node arm_positions = YAML::LoadFile(yaml_file);
    std::vector<float> pose = arm_positions["dropping_position"].as<std::vector<float>>();

    drop_pose_.header.frame_id = BASE_FRAME;
    // drop_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(ba_helper::ConvertDegreesToRadians(pose[3]), ba_helper::ConvertDegreesToRadians(pose[4]), ba_helper::ConvertDegreesToRadians(pose[5]));

    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(ba_helper::ConvertDegreesToRadians(pose[3]), ba_helper::ConvertDegreesToRadians(pose[4]), ba_helper::ConvertDegreesToRadians(pose[5]));
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

    drop_pose_.pose.orientation = msg_quat;
    drop_pose_.pose.position.x = pose[0];
    drop_pose_.pose.position.y = pose[1];
    drop_pose_.pose.position.z = pose[2];
}

#pragma endregion

#pragma endregion
#include "manipulator_behaviors.h"

// #pragma region DequeueTomato

#pragma region public

/**
 * @brief Construct a new Dequeue Tomato:: Dequeue Tomato object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */
// DequeueTomato::DequeueTomato(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config)
// {
//     // ROS_LOG_INIT(this->name().c_str());
// }

/**
 * @brief Set the TomatoQueue reference object
 * 
 * @param path_queue Reference to the TomatoQueue 
 */
// void DequeueTomato::init(TomatoQueue &tqueue)
// {
//     // tomato_queue_ = &tqueue;
// }

/**
 * @brief Handles the tick from the behavior tree
 *
 * @return BT::NodeStatus The status of the node
 */
// BT::NodeStatus DequeueTomato::tick()
// {
//     // TomatoCoordinates next_tomato = tomato_queue_->GetNextReachableTomato();
//     // setOutput("target_x", next_tomato.x);
//     // setOutput("target_y", next_tomato.y);
//     // setOutput("target_z", next_tomato.z);

//     // float airgap = ba_helper::CalculateGripperParameterFromDesiredAirgap(2 * next_tomato.r);
//     // setOutput("gripper_airgap", (float)airgap);

//     return BT::NodeStatus::SUCCESS;
// }

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
// BT::PortsList DequeueTomato::providedPorts()
// {
//     return {BT::OutputPort<float>("target_x"),
//             BT::OutputPort<float>("target_y"),
//             BT::OutputPort<float>("target_z"),
//             BT::OutputPort<int>("num_tomatoes"),
//             BT::OutputPort<float>("gripper_airgap")};
// }

#pragma endregion

#pragma region private

/**
 * @brief Transforms the coordinates of a tomato to a pose for the robot
 * 
 * @param tomato The location of the tomato to grasp
 * @param frame The selected frame
 * @return geometry_msgs::PoseStamped The robots pose
 */
// geometry_msgs::msg::PoseStamped DequeueTomato::TransformTomatoToPose(TomatoCoordinates tomato, std::string frame)
// {
//     geometry_msgs::msg::PoseStamped result;
//     result.header.frame_id = frame;
//     // result.header.stamp = ros::Time();
//     result.pose.position.x = tomato.x;
//     result.pose.position.y = tomato.y;
//     result.pose.position.z = tomato.z;
//     // result.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
//     return result;
// }

#pragma endregion

#pragma endregion

// #pragma region FilterTomatoQueue

/**
 * @brief Construct a new Filter Tomato Queue:: Filter Tomato Queue object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */
// FilterTomatoQueue::FilterTomatoQueue(const std::string &name, const BT::NodeConfiguration &config)
//     : BT::SyncActionNode(name, config)
// {
//     // ROS_LOG_INIT(this->name().c_str());
// }

/**
 * @brief Set the TomatoQueue reference object
 * 
 * @param path_queue Reference to the TomatoQueue 
 */
// void FilterTomatoQueue::init(TomatoQueue &tomato_queue)
// {
//     tomato_queue_ = &tomato_queue;
// }

/**
 * @brief Handles the tick from the behavior tree
 *
 * @return BT::NodeStatus The status of the node
 */
// BT::NodeStatus FilterTomatoQueue::tick()
// {
//     LOG_MANI_START(this->name());
//     geometry_msgs::msg::PoseStamped manipulator_base_local;
//     geometry_msgs::msg::PoseStamped manipulator_base_map;
//     // tf::TransformListener listener;
//     // listener.waitForTransform(MAP_FRAME, ARM_BASE_FRAME, ros::Time(0), ros::Duration(3.0));
//     manipulator_base_local.pose.position.x =
//         manipulator_base_local.pose.position.y =
//             manipulator_base_local.pose.position.z = 0.0;
//     manipulator_base_local.header.frame_id = ARM_BASE_FRAME;
//     // manipulator_base_local.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
//     // manipulator_base_local.header.stamp = ros::Time();
//     //listener.transformPose(MAP_FRAME, manipulator_base_local, manipulator_base_map);
//     // int number_of_reachables = tomato_queue_->GetReachableTomatoCount(manipulator_base_map.pose.position.x,
//     //                                                        manipulator_base_map.pose.position.y,
//     //                                                        manipulator_base_map.pose.position.z, UR5_WORKING_RADIUS);

//     // ROS_INFO("[%s] Printing queue!", this->name().c_str());
//     // tomato_queue_->PrintQueue();
//     // if (number_of_reachables == 0)
//     // {
//         // ROS_ERROR("[%s] No more locations!", this->name().c_str());
//     //     LOG_MANI_STOP(this->name());
//     //    return BT::NodeStatus::FAILURE;
//     // }
//     // ROS_INFO("Currently reachable tomatoes: %d", number_of_reachables);
//     // setOutput("num_tomatoes", number_of_reachables);
//     LOG_MANI_STOP(this->name());
//     return BT::NodeStatus::SUCCESS;
// }

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
// BT::PortsList FilterTomatoQueue::providedPorts()
// {
//     return {BT::OutputPort<int>("num_tomatoes")};
// }

#pragma endregion

#pragma region ManipulatorGraspTomato

/**
 * @brief Construct a new Manipulator Grasp Tomato:: Manipulator Grasp Tomato object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */
ManipulatorGraspTomato::ManipulatorGraspTomato(const std::string &name, const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config),
      client_(rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(rclcpp::Node::make_shared(""), "/summit_xl/move_group", nullptr, rcl_action_client_get_default_options()))
{
    client_->wait_for_action_server();
    ROS_LOG_INIT(this->name().c_str());
}

/**
 * @brief Set the Manipulator object
 * 
 * @param manipulator The manipulator
 */
void ManipulatorGraspTomato::init(Manipulator manipulator)
{
    manipulator_ = manipulator;
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorGraspTomato::onStart()
{
    LOG_MANI_START(this->name());
    BT::Expected<float> tomato_map_x = getInput<float>("target_x");
    BT::Expected<float> tomato_map_y = getInput<float>("target_y");
    BT::Expected<float> tomato_map_z = getInput<float>("target_z");
    if (!tomato_map_x || !tomato_map_y || !tomato_map_z)
    {
        // ROS_ERROR("GOT NO POSE!");
        return BT::NodeStatus::FAILURE;
    }
    // geometry_msgs::msg::Point armlink_loc = ba_helper::GetCurrentArmbaseLocation();
    // float phi = atan2(tomato_map_y.value() - armlink_loc.y, tomato_map_x.value() - armlink_loc.x);
    // geometry_msgs::msg::PoseStamped tomato;
    // tomato.header.frame_id = MAP_FRAME;
    // tomato.pose.position.x = tomato_map_x.value() - sin(phi) * 0;
    // tomato.pose.position.y = tomato_map_y.value() - cos(phi) * 0;
    // tomato.pose.position.z = tomato_map_z.value();
    // // tomato.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

    // // manipulator_.MoveGripperToTomato(tomato);
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorGraspTomato::onRunning()
{
    BT::NodeStatus state = manipulator_.GetNodeStatus(this->name().c_str());
    if (state != BT::NodeStatus::RUNNING)
    {   
        LOG_MANI_STOP(this->name());
    }
    return state;

}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorGraspTomato::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorGraspTomato::providedPorts()
{
    return {BT::InputPort<float>("target_x"), BT::InputPort<float>("target_y"), BT::InputPort<float>("target_z")};
}

#pragma endregion

#pragma region ManipulatorPregrasp

/**
 * @brief Construct a new Manipulator Pregrasp:: Manipulator Pregrasp object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */
ManipulatorPregrasp::ManipulatorPregrasp(const std::string &name, const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config)
{
    ROS_LOG_INIT(this->name().c_str());
}

/**
 * @brief Set the Manipulator object
 * 
 * @param manipulator The manipulator 
 */
void ManipulatorPregrasp::init(Manipulator manipulator)
{
    manipulator_ = manipulator;
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPregrasp::onStart()
{
    LOG_MANI_START(this->name());
    BT::Expected<float> tomato_map_x = getInput<float>("target_x");
    BT::Expected<float> tomato_map_y = getInput<float>("target_y");
    BT::Expected<float> tomato_map_z = getInput<float>("target_z");
    BT::Expected<float> pregresp_offset = getInput<float>("pregrasp_offset");
    if (!tomato_map_x || !tomato_map_y || !tomato_map_z)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ManipulatorPregrasp"), "GOT NO POSE!");
        return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PoseStamped tomato;
    tomato.header.frame_id = MAP_FRAME;
    geometry_msgs::msg::Point armlink_loc = ba_helper::GetCurrentArmbaseLocation();
    float phi = atan2(tomato_map_y.value() - armlink_loc.y, tomato_map_x.value() - armlink_loc.x);
    tomato.pose.position.x = tomato_map_x.value() - sin(phi) * 0;
    tomato.pose.position.y = tomato_map_y.value() - cos(phi) * 0;
    tomato.pose.position.z = tomato_map_z.value();

    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0, 0, 0);
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);
    tomato.pose.orientation = msg_quat;

    manipulator_.MoveGripperToPregraspPose(tomato, pregresp_offset.value());
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPregrasp::onRunning()
{
    BT::NodeStatus state = manipulator_.GetNodeStatus(this->name().c_str());
    if (state != BT::NodeStatus::RUNNING)
    {   
        LOG_MANI_STOP(this->name());
    }
    return state;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorPregrasp::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorPregrasp::providedPorts()
{
    return {BT::InputPort<float>("target_x"),
            BT::InputPort<float>("target_y"),
            BT::InputPort<float>("target_z"),
            BT::InputPort<float>("pregrasp_offset")};
}

#pragma endregion

#pragma region ManipulatorPostgraspRetreat

/**
 * @brief Construct a new Manipulator Postgrasp Retreat:: Manipulator Postgrasp Retreat object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */
ManipulatorPostgraspRetreat::ManipulatorPostgraspRetreat(const std::string &name, const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config)
{
    ROS_LOG_INIT(this->name().c_str());
}

/**
 * @brief Set the Manipulator object
 * 
 * @param manipulator The manipulator
 */
void ManipulatorPostgraspRetreat::init(Manipulator manipulator)
{
    manipulator_ = manipulator;
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPostgraspRetreat::onStart()
{
    LOG_MANI_START(this->name());
    manipulator_.MoveLinearVec(0, 0, 0.12);
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPostgraspRetreat::onRunning()
{   
    BT::NodeStatus state = manipulator_.GetNodeStatus(this->name().c_str());
    if (state != BT::NodeStatus::RUNNING)
    {   
        LOG_MANI_STOP(this->name());
    }
    return state;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorPostgraspRetreat::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorPostgraspRetreat::providedPorts()
{
    return {BT::InputPort<float>("target_x"),
            BT::InputPort<float>("target_y"),
            BT::InputPort<float>("target_z")};
}

#pragma endregion

#pragma region ManipulatorDropTomato

/**
 * @brief Construct a new Manipulator Drop Tomato:: Manipulator Drop Tomato object
 * 
 * @param name The name of the behavior
 */
ManipulatorDropTomato::ManipulatorDropTomato(const std::string &name)
    : BT::StatefulActionNode(name, {})
{
    ROS_LOG_INIT(this->name().c_str());
}

/**
 * @brief Set the Manipulator object
 * 
 * @param manipulator The manipulator
 */
void ManipulatorDropTomato::init(Manipulator manipulator)
{
    manipulator_ = manipulator;
}

/**
 * @brief Set the reference to the TomatoQueue object
 * 
 * @param tomato_queue The reference to the TomatoQueue
 */
// void ManipulatorDropTomato::init(TomatoQueue &tomato_queue)
// {
//     tomato_queue_ = &tomato_queue;
// }

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorDropTomato::onStart()
{
    LOG_MANI_START(this->name());
    // manipulator_.DropTomatoInBasket();
    // ROS_INFO("going to drop tomato in basket");
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorDropTomato::onRunning()
{
    BT::NodeStatus state = manipulator_.GetNodeStatus(this->name().c_str());
    if (state == BT::NodeStatus::SUCCESS)
    {   
        // tomato_queue_->SetTomatoAsPicked();
        // tomato_queue_->AddTomatoToBasketQueue();
    }
    if(state != BT::NodeStatus::RUNNING){
        LOG_MANI_STOP(this->name());
    }
    return state;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorDropTomato::onHalted() {}


#pragma endregion

#pragma region ManipulatorScanPose

/**
 * @brief Construct a new Manipulator Scan Pose:: Manipulator Scan Pose object
 * 
 * @param name The name of the behavior
 */
ManipulatorScanPose::ManipulatorScanPose(const std::string &name)
    : BT::StatefulActionNode(name, {})
{
    ROS_LOG_INIT(this->name().c_str());
}

/**
 * @brief Set the Manipulator object
 * 
 * @param manipulator The manipulator
 */
void ManipulatorScanPose::init(Manipulator manipulator)
{
    manipulator_ = manipulator;
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorScanPose::onStart()
{
    LOG_MANI_START(this->name());
    manipulator_.MoveToScanningPosition();
    // ROS_INFO("moving EE to scan position");
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorScanPose::onRunning()
{
    BT::NodeStatus state = manipulator_.GetNodeStatus(this->name().c_str());
    if (state != BT::NodeStatus::RUNNING)
    {   
        LOG_MANI_STOP(this->name());
    }
    return state;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorScanPose::onHalted() {}

#pragma endregion
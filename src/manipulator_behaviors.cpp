#include "manipulator_behaviors.h"

#pragma region ManipulatorGrasp

/**
 * @brief Construct a new Manipulator Grasp :: Manipulator Grasp object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */

ManipulatorGrasp::ManipulatorGrasp(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
{
    action_name = this->name();
    
    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s] Node shared pointer was passed!", action_name.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] Initialized!", action_name.c_str());
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorGrasp::onStart()
{
    LOG_MANI_START(action_name);

    RCLCPP_INFO(node_->get_logger(), "action start: %s", action_name.c_str());

    BT::Optional<double> base_footprint_x = getInput<double>("target_x");
    BT::Optional<double> base_footprint_y = getInput<double>("target_y");
    BT::Optional<double> base_footprint_z = getInput<double>("target_z");
    BT::Optional<double> base_footprint_roll = getInput<double>("target_roll");
    BT::Optional<double> base_footprint_pitch = getInput<double>("target_pitch");
    BT::Optional<double> base_footprint_yaw = getInput<double>("target_yaw");
    // BT::Optional<double> tcp_offset_xy = getInput<double>("tcp_offset_xy"); 
    BT::Optional<double> tcp_offset_x = getInput<double>("tcp_offset_x"); 
    BT::Optional<double> tcp_offset_y = getInput<double>("tcp_offset_y"); 
    BT::Optional<double> tcp_offset_z = getInput<double>("tcp_offset_z");

    RCLCPP_INFO(node_->get_logger(), "[%s] moving gripper to target linearly", action_name.c_str());
    
    // manipulator_.MoveGripperToPoseLinear(base_footprint_x.value(), base_footprint_y.value(), base_footprint_z.value(), base_footprint_roll.value(), base_footprint_pitch.value(), base_footprint_yaw.value(), tcp_offset_xy.value(), tcp_offset_z.value());

    manipulator_.MoveGripperToPoseLinear(base_footprint_x.value(), base_footprint_y.value(), base_footprint_z.value(), base_footprint_roll.value(), base_footprint_pitch.value(), base_footprint_yaw.value(), tcp_offset_x.value(), tcp_offset_y.value(), tcp_offset_z.value());
    
    RCLCPP_INFO(node_->get_logger(), "[%s] moved gripper to target linearly", action_name.c_str());
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorGrasp::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorGrasp::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorGrasp::providedPorts()
{
    // return {BT::InputPort<double>("target_x"), BT::InputPort<double>("target_y"), BT::InputPort<double>("target_z"), BT::InputPort<double>("target_roll"), BT::InputPort<double>("target_pitch"), BT::InputPort<double>("target_yaw"), BT::InputPort<double>("tcp_offset_xy"), BT::InputPort<double>("tcp_offset_z")};

    return {BT::InputPort<double>("target_x"), BT::InputPort<double>("target_y"), BT::InputPort<double>("target_z"), BT::InputPort<double>("target_roll"), BT::InputPort<double>("target_pitch"), BT::InputPort<double>("target_yaw"), BT::InputPort<double>("tcp_offset_x"), BT::InputPort<double>("tcp_offset_y"), BT::InputPort<double>("tcp_offset_z")};
}

#pragma endregion

#pragma region ManipulatorPregraspPlan

/**
 * @brief Construct a new Manipulator ManipulatorPregraspPlan:: ManipulatorPregraspPlan Pregrasp object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */
ManipulatorPregraspPlan::ManipulatorPregraspPlan(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
{
    action_name = this->name();
    
    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s] Node shared pointer was passed!", action_name.c_str());
    }

    const rclcpp::QoS feedback_sub_qos = rclcpp::QoS(1);

    trajectory_execute_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/summit/trajectory_execute", feedback_sub_qos, std::bind(&ManipulatorPregraspPlan::topic_callback, this, _1));

    wait_set_.add_subscription(trajectory_execute_subscription_);

    // auto topic_callback =
    //   [this](std_msgs::msg::Bool::SharedPtr msg) -> void {

    //     if (flag_ == true)
    //     {
    //         RCLCPP_ERROR(node_->get_logger(), "flag_ already true, returning!");
    //         flag_ = false;
    //         trajectory_execute_ = false;
    //         return;
    //     }
        
    //     flag_ = true;
    //     trajectory_execute_ = msg->data;
    //     RCLCPP_INFO(node_->get_logger(), "Received trajectory execute (0: False, 1: True): %d", msg->data);
    //   };
    
    // trajectory_execute_subscription_ = node_->create_subscription<std_msgs::msg::Bool>("/summit/trajectory_execute", feedback_sub_qos, topic_callback);

    RCLCPP_INFO(node_->get_logger(), "[%s] Initialized!", action_name.c_str());
}

void ManipulatorPregraspPlan::topic_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // if (flag_ == true)
    // {
    //     RCLCPP_ERROR(node_->get_logger(), "flag_ already true, returning!");
    //     flag_ = false;
    //     trajectory_execute_ = false;
    //     return;
    // }
    
    // flag_ = true;
    // trajectory_execute_ = msg->data;
    // RCLCPP_INFO(node_->get_logger(), "Received trajectory execute (0: False, 1: True): %d", trajectory_execute_);

    return;
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPregraspPlan::onStart()
{
    LOG_MANI_START(action_name);

    RCLCPP_INFO(node_->get_logger(), "action start: %s", action_name.c_str());
    
    BT::Optional<bool> pose_from_tf = getInput<bool>("pose_from_tf");
    BT::Optional<double> base_footprint_x = getInput<double>("target_x");
    BT::Optional<double> base_footprint_y = getInput<double>("target_y");
    BT::Optional<double> base_footprint_z = getInput<double>("target_z");
    BT::Optional<double> base_footprint_roll = getInput<double>("target_roll");
    BT::Optional<double> base_footprint_pitch = getInput<double>("target_pitch");
    BT::Optional<double> base_footprint_yaw = getInput<double>("target_yaw");
    BT::Optional<double> pregresp_offset = getInput<double>("pregrasp_offset");

    setOutput<double>("target_x_cp", base_footprint_x.value());
    setOutput<double>("target_y_cp", base_footprint_y.value());
    setOutput<double>("target_z_cp", base_footprint_z.value());
    setOutput<double>("target_roll_cp", base_footprint_roll.value());
    setOutput<double>("target_pitch_cp", base_footprint_pitch.value());
    setOutput<double>("target_yaw_cp", base_footprint_yaw.value());

    std::string target_frame;

    RCLCPP_INFO(node_->get_logger(), "[%s] pregrasp started", action_name.c_str());

    int no_of_deploy_sensors_{0};
    getInput<int>("no_of_deploy_sensors", no_of_deploy_sensors_);
    
    if (no_of_deploy_sensors_ != 0)
    {
        std::string sensor_deploy_frame_names_dynamic_;

        getInput<std::string>("sensor_deploy_frame_names_dynamic", sensor_deploy_frame_names_dynamic_);

        int pos_comma = sensor_deploy_frame_names_dynamic_.find(",");

        setOutput<int>("pos_comma", pos_comma);

        target_frame = sensor_deploy_frame_names_dynamic_.substr(0, pos_comma);

        plan_trajectory_ = manipulator_.PlanGripperToPose(pose_from_tf.value(), target_frame, base_footprint_x.value(), base_footprint_y.value(), base_footprint_z.value(), base_footprint_roll.value(), base_footprint_pitch.value(), base_footprint_yaw.value(), pregresp_offset.value());

        setOutput<moveit_msgs::msg::RobotTrajectory>("plan_trajectory", plan_trajectory_);

        RCLCPP_INFO(node_->get_logger(), "[%s] pregrasp plan finished", action_name.c_str());

        // sensor_deploy_frame_names_dynamic_.erase(0, pos_comma + 1);
        
        // setOutput<std::string>("sensor_deploy_frame_names_dynamic", sensor_deploy_frame_names_dynamic_);

        // int no_of_deploy_sensors_dynamic = std::count(sensor_deploy_frame_names_dynamic_.begin(), sensor_deploy_frame_names_dynamic_.end(), ',');

        // RCLCPP_INFO(node_->get_logger(), "%d sensors already deployed!", no_of_deploy_sensors_ - no_of_deploy_sensors_dynamic); 
        // RCLCPP_INFO(node_->get_logger(), "%d sensors yet to be deployed!", no_of_deploy_sensors_dynamic);
    }

    else
    {
        target_frame = "liquid";
        
        plan_trajectory_ = manipulator_.PlanGripperToPose(pose_from_tf.value(), target_frame, base_footprint_x.value(), base_footprint_y.value(), base_footprint_z.value(), base_footprint_roll.value(), base_footprint_pitch.value(), base_footprint_yaw.value(), pregresp_offset.value());

        setOutput<moveit_msgs::msg::RobotTrajectory>("plan_trajectory", plan_trajectory_);

        RCLCPP_INFO(node_->get_logger(), "[%s] pregrasp plan finished", action_name.c_str());
    }
    
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPregraspPlan::onRunning()
{   
    int traj_exec_wait_min{5};
    
    if (!plan_trajectory_.joint_trajectory.header.frame_id.empty())
    {
        if (count_ == 0)
        {
            RCLCPP_WARN(node_->get_logger(), "[%s] valid trajectory plan received!", action_name.c_str());
            
            RCLCPP_WARN(node_->get_logger(), "[%s] check: Execute Trajectory? Waiting for publisher for about %d mins max., Format: $ ros2 topic pub /summit/trajectory_execute std_msgs/msg/Bool \"data: true\" --once", action_name.c_str(), traj_exec_wait_min);
            count_++;
            return BT::NodeStatus::RUNNING;
        }

        // RCLCPP_INFO(node_->get_logger(), "count_ = %d", count_);

        // RCLCPP_INFO(node_->get_logger(), "flag_ = %d", flag_);
        
        int timeout_secs = traj_exec_wait_min * 60;
        
        if (count_ <= timeout_secs)
        {
            // Wait for the subscriber event to trigger. Set a 1 ms margin to trigger a timeout.
            const auto wait_result = wait_set_.wait(std::chrono::milliseconds(1001));
            switch (wait_result.kind()) {
                case rclcpp::WaitResultKind::Ready:
                {
                    std_msgs::msg::Bool take_msg;
                    rclcpp::MessageInfo msg_info;
                    if (trajectory_execute_subscription_->take(take_msg, msg_info)) {
                        bool value_received = take_msg.data;
                        RCLCPP_WARN(node_->get_logger(), "[%s] check: Received take trajectory execute (0: False, 1: True): %d", action_name.c_str(), value_received);

                        setOutput<bool>("execute_trajectory", value_received);
                        
                        if (value_received)
                        {
                            RCLCPP_WARN(node_->get_logger(), "[%s] check: Trajectory execute approved", action_name.c_str());

                            value_received = false;
                            
                            return BT::NodeStatus::SUCCESS;
                        }

                        else
                        {
                            RCLCPP_ERROR(node_->get_logger(), "[%s] check: Trajectory execute disapproved", action_name.c_str());
                            return BT::NodeStatus::FAILURE;
                        }
                    }
                    break;
                }

                case rclcpp::WaitResultKind::Timeout:
                {
                    if (rclcpp::ok()) {
                        RCLCPP_WARN(node_->get_logger(), "[%s] check: Timeout. No message received yet, still waiting for about %d secs", action_name.c_str(), timeout_secs - count_);
                    }
                    break;
                }

                default:
                {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] check: Error. Wait-set failed.", action_name.c_str());
                }
            }
            
            // if (flag_)
            // {
            //     setOutput<bool>("execute_trajectory", trajectory_execute_);
                
            //     if (trajectory_execute_)
            //     {
            //         RCLCPP_WARN(node_->get_logger(), "check: Trajectory execute approved");

            //         trajectory_execute_ = false;
                    
            //         return BT::NodeStatus::SUCCESS;
            //     }

            //     else
            //     {
            //         RCLCPP_ERROR(node_->get_logger(), "Trajectory execute disapproved");
            //         return BT::NodeStatus::FAILURE;
            //     }

            // flag_ = false;
            // }
        }
        
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Timeout! no response received, returning failure", action_name.c_str());
            return BT::NodeStatus::FAILURE;
        }

        count_++;
    }

    else
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] invalid trajectory plan received, returning failure", action_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorPregraspPlan::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorPregraspPlan::providedPorts()
{
    return {BT::InputPort<std::string>("sensor_deploy_frame_names_dynamic"), BT::OutputPort<int>("pos_comma"), BT::InputPort<int>("no_of_deploy_sensors"), BT::InputPort<bool>("pose_from_tf"), BT::InputPort<double>("target_x"), BT::InputPort<double>("target_y"), BT::InputPort<double>("target_z"), BT::InputPort<double>("pregrasp_offset"), BT::InputPort<double>("target_roll"), BT::InputPort<double>("target_pitch"), BT::InputPort<double>("target_yaw"), BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("plan_trajectory"), BT::OutputPort<bool>("execute_trajectory"), BT::OutputPort<double>("target_x_cp"), BT::OutputPort<double>("target_y_cp"), BT::OutputPort<double>("target_z_cp"), BT::OutputPort<double>("target_roll_cp"), BT::OutputPort<double>("target_pitch_cp"), BT::OutputPort<double>("target_yaw_cp")};
}

#pragma endregion

#pragma region ManipulatorPregraspExecute

/**
 * @brief Construct a new Manipulator Pregrasp Execute:: Manipulator Pregrasp Execute object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */
ManipulatorPregraspExecute::ManipulatorPregraspExecute(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
{   
    action_name = this->name();
    
    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s] Node shared pointer was passed!", action_name.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] Initialized!", action_name.c_str());
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPregraspExecute::onStart()
{
    LOG_MANI_START(action_name);

    RCLCPP_INFO(node_->get_logger(), "action start: %s", action_name.c_str());

    bool execute_trajectory{false};
    getInput<bool>("execute_trajectory", execute_trajectory);
    
    if (execute_trajectory)
    {
        RCLCPP_INFO(node_->get_logger(), "[%s] signaled to execute trajectory, executing!", action_name.c_str());
        
        BT::Optional<moveit_msgs::msg::RobotTrajectory> trajectory = getInput<moveit_msgs::msg::RobotTrajectory>("plan_trajectory");

        moveit::core::MoveItErrorCode error_code = manipulator_.ExecuteGripperToPose(trajectory.value());

        error_message_ = moveit::core::error_code_to_string(error_code);

        RCLCPP_INFO(node_->get_logger(), "[%s] Error message: %s", action_name.c_str(), error_message_.c_str());

        int no_of_deploy_sensors_{0};
        getInput<int>("no_of_deploy_sensors", no_of_deploy_sensors_);
        
        if (error_message_ == "SUCCESS" && no_of_deploy_sensors_ != 0)
        {
            std::string sensor_deploy_frame_names_dynamic_;

            getInput<std::string>("sensor_deploy_frame_names_dynamic", sensor_deploy_frame_names_dynamic_);

            int pos_comma{0};
            getInput<int>("pos_comma", pos_comma);
            
            sensor_deploy_frame_names_dynamic_.erase(0, pos_comma + 1);
                
            setOutput<std::string>("sensor_deploy_frame_names_dynamic", sensor_deploy_frame_names_dynamic_);

            int no_of_deploy_sensors_dynamic = std::count(sensor_deploy_frame_names_dynamic_.begin(), sensor_deploy_frame_names_dynamic_.end(), ',');

            RCLCPP_INFO(node_->get_logger(), "[%s] %d sensors already deployed!", action_name.c_str(), no_of_deploy_sensors_ - no_of_deploy_sensors_dynamic); 
            RCLCPP_INFO(node_->get_logger(), "[%s] %d sensors yet to be deployed!", action_name.c_str(), no_of_deploy_sensors_dynamic);
        }

        execute_trajectory = false;
    }

    else
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] signaled to not execute trajectory, returning failure", action_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPregraspExecute::onRunning()
{
    if (error_message_ == "SUCCESS")
    {
        return BT::NodeStatus::SUCCESS;
    }
    
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorPregraspExecute::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorPregraspExecute::providedPorts()
{
    return {BT::InputPort<moveit_msgs::msg::RobotTrajectory>("plan_trajectory"), BT::BidirectionalPort<std::string>("sensor_deploy_frame_names_dynamic"), BT::InputPort<int>("pos_comma"), BT::InputPort<int>("no_of_deploy_sensors"), BT::InputPort<bool>("execute_trajectory")};
}

#pragma endregion

#pragma region ManipulatorPostgraspRetreat

/**
 * @brief Construct a new Manipulator Postgrasp Retreat:: Manipulator Postgrasp Retreat object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */
ManipulatorPostgraspRetreat::ManipulatorPostgraspRetreat(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
{
    action_name = this->name();

    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s] Node shared pointer was passed!", action_name.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] Initialized!", action_name.c_str());
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPostgraspRetreat::onStart()
{
    LOG_MANI_START(action_name);

    RCLCPP_INFO(node_->get_logger(), "action start: %s", action_name.c_str());

    RCLCPP_INFO(node_->get_logger(), "[%s] post grasp started", action_name.c_str());
    
    BT::Optional<double> add_pos_z = getInput<double>("add_pos_z");
    
    // manipulator_.MoveLinearVec(0, 0, 0.12);
    manipulator_.MoveLinearVec(0, 0, add_pos_z.value());
    
    RCLCPP_INFO(node_->get_logger(), "[%s] post grasp finished", action_name.c_str());
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPostgraspRetreat::onRunning()
{   
    return BT::NodeStatus::SUCCESS;
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
    // RCLCPP_WARN(rclcpp::get_logger("ManipulatorPostgraspRetreat"), "returning empty BT::PortsList!");
    return {BT::InputPort<double>("add_pos_z")};
}

#pragma endregion

#pragma region ManipulatorDrop

/**
 * @brief Construct a new Manipulator Drop :: Manipulator Drop an object
 * 
 * @param name The name of the behavior
 */
ManipulatorDrop::ManipulatorDrop(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
{
    action_name = this->name();

    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s] Node shared pointer was passed!", action_name.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] Initialized!", action_name.c_str());
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorDrop::onStart()
{
    LOG_MANI_START(action_name);

    RCLCPP_INFO(node_->get_logger(), "action start: %s", action_name.c_str());

    RCLCPP_INFO(node_->get_logger(), "[%s]: going to drop object", action_name.c_str());
    manipulator_.DropObject();
    RCLCPP_INFO(node_->get_logger(), "[%s]: object dropped", action_name.c_str());
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorDrop::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorDrop::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorDrop::providedPorts()
{
    RCLCPP_WARN(rclcpp::get_logger("ManipulatorDrop"), "returning empty BT::PortsList!");
    return {};
}


#pragma endregion

#pragma region ManipulatorScanPose

/**
 * @brief Construct a new Manipulator Scan Pose:: Manipulator Scan Pose object
 * 
 * @param name The name of the behavior
 */
ManipulatorScanPose::ManipulatorScanPose(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
{
    action_name = this->name();
    
    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s] Node shared pointer was passed!", action_name.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] Initialized!", action_name.c_str());
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorScanPose::onStart()
{
    LOG_MANI_START(action_name);
    RCLCPP_INFO(node_->get_logger(), "[%s] moving EE to scan position", action_name.c_str());
    manipulator_.MoveToScanningPosition();
    RCLCPP_INFO(node_->get_logger(), "[%s] moved EE to scan position", action_name.c_str());
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorScanPose::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorScanPose::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorScanPose::providedPorts()
{
    RCLCPP_WARN(rclcpp::get_logger("ManipulatorScanPose"), "returning empty BT::PortsList!");
    return {};
}

#pragma endregion
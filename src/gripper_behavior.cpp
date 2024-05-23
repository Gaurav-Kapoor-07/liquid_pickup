#include "gripper_behavior.h"

#pragma region public

/**
 * @brief Construct a new Gripper Actuator:: Gripper Actuator object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */
GripperActuator::GripperActuator(const std::string &name, const BT::NodeConfiguration &config):BT::SyncActionNode(name, config){
    object_attached_ = false;
    object_attached_before_ = false;
    // ROS_LOG_INIT(this->name().c_str());
}

/**
 * @brief Handles the tick from the behavior tree
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus GripperActuator::tick(){
    // ros::NodeHandle nh;
    // grasp_publisher_ = nh.advertise<std_msgs::Float64>(GRIPPER_TOPIC, 100, true);
    // grasp_subscriber_ = nh.subscribe(DEFAULT_CONTACT_TOPIC, 1, &GripperActuator::ResultCallback, this);
    // std_msgs::Float64 payload;
    // float gripper_val = getInput<float>("gripper_param").value();
    // ROS_INFO("Gripping -> value: %.3f", gripper_val);
    // payload.data = gripper_val;
    // grasp_publisher_.publish(payload);
    // ros::Duration(0.2).sleep();
    BT::NodeStatus result = object_attached_^object_attached_before_?BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    object_attached_before_ = object_attached_;
    return BT::NodeStatus::SUCCESS;
};

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList GripperActuator::providedPorts(){
    return {BT::InputPort<float>("gripper_param")} ;
}

#pragma endregion

#pragma region private

/**
 * @brief Callback method which is executed when a message appears into a subscribed topic
 * 
 * @param message The message
 */
void GripperActuator::ResultCallback(const std_msgs::msg::Float64 &message){
    object_attached_before_ = object_attached_;
    object_attached_ = !object_attached_;
}

#pragma endregion
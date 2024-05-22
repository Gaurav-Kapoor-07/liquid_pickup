#include "navigation_behaviors.h"

#pragma region SetLocations

#pragma region public

/**
 * @brief Set the Locations:: Set Locations object
 *
 * @param name The name of the behavior
 * @param config The node configuration
 */
SetLocations::SetLocations(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config)
{
    // ROS_LOG_INIT(this->name().c_str());
}

/**
 * @brief Set the Locations::init object
 *
 * @param path_queue Reference to the PathQueue
 */
void SetLocations::init(PathQueue &path_queue)
{
    path_queue_ = &path_queue;
}

/**
 * @brief Handles the tick from the behavior tree
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus SetLocations::tick()
{
    // Read YAML file
    std::string yaml_file;
    // ros::param::get("location_file", yaml_file);
    // ROS_INFO("%s", yaml_file.c_str());
    YAML::Node locations = YAML::LoadFile(yaml_file)["path_locations"];
    for (YAML::const_iterator it = locations.begin(); it != locations.end(); ++it)
    {
        std::array<float, 3> entry = it->second.as<std::array<float, 3>>();
        PathLocation location = {entry[0], entry[1], entry[2]};
        path_queue_->Enqueue(QueueSide::back, location);
    }

    int num_locs = path_queue_->GetQueue().size();
    if (num_locs == 0)
    {
        // ROS_ERROR("[%s] No path locations found!", this->name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    setOutput("num_locs", num_locs);
    // ROS_INFO("[%s] Found %d locations.", this->name().c_str(), num_locs);

    YAML::Node fixed_locations = YAML::LoadFile(yaml_file)["fixed_locations"];
    std::array<float, 3> entry_basket_change_location = (fixed_locations["empty_basket_location"]).as<std::array<float, 3>>();
    PathLocation basket_change_location = {entry_basket_change_location[0], entry_basket_change_location[1], entry_basket_change_location[2]};
    path_queue_->SetEmptyBasketLocation(basket_change_location);

    std::array<float, 3> entry_recharge_location = (fixed_locations["recharge_location"]).as<std::array<float, 3>>();
    PathLocation recharge_location = {entry_recharge_location[0], entry_recharge_location[1], entry_recharge_location[2]};
    path_queue_->SetRechargeLocation(recharge_location);

    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList SetLocations::providedPorts()
{
    return {BT::OutputPort<int>("num_locs")};
}

#pragma endregion

#pragma endregion

#pragma region GetLocationFromQueue

#pragma region public methods

/**
 * @brief Get the Location From Queue:: Get Location From Queue object
 *
 * @param name The name of the behavior
 * @param config The node configuration
 */
GetLocationFromQueue::GetLocationFromQueue(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config)
{
    // ROS_LOG_INIT(this->name().c_str());
}

/**
 * @brief Set the Locations::init object
 *
 * @param path_queue Reference to the PathQueue
 */
void GetLocationFromQueue::init(PathQueue &path_queue)
{
    path_queue_ = &path_queue;
}

/**
 * @brief Handles the tick from the behavior tree
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus GetLocationFromQueue::tick()
{
    if (path_queue_->GetQueue().empty())
    {
        // ROS_ERROR("[%s] No more locations!", this->name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    else
    {
        PathLocation target_location = path_queue_->GetQueue().front();
        setOutput("target_location", target_location);
        path_queue_->Dequeue(QueueSide::front);
        return BT::NodeStatus::SUCCESS;
    }
}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList GetLocationFromQueue::providedPorts()
{
    return {BT::OutputPort<PathLocation>("target_location")};
}

#pragma endregion

#pragma endregion

#pragma region GoToPose

#pragma region public methods
/**
 * @brief Construct a new Go To Pose:: Go To Pose object
 *
 * @param name The name of the behavior
 * @param config The node configuration
 */
// GoToPose::GoToPose(const std::string &name, const BT::NodeConfiguration &config) : BT::StatefulActionNode(name, config),
//                                                                                    client_("/summit_xl/move_base", true)
// {
//     client_.waitForServer();
//     std::string yaml_file;
//     ros::param::get("location_file", yaml_file);
//     locations_ = YAML::LoadFile(yaml_file)["path_locations"];
//     ROS_LOG_INIT(this->name().c_str());
// }

/**
 * @brief Set the Manipulator object
 *
 * @param manipulator The Manipulator
 */
void GoToPose::init(Manipulator manipulator)
{
    manipulator_ = manipulator;
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus GoToPose::onStart()
{
    // LOG_NAV_START(this->name());
    // manipulator_.MoveToDrivingPosition();
    // BT::Optional<PathLocation> loc = getInput<PathLocation>("loc");
    // ROS_INFO("[%s] next goal: x=%.2f y=%.2f w=%.2f", this->name().c_str(), loc->x, loc->y, loc->orientation);
    // goal_.target_pose.header.frame_id = "summit_xl_map";
    // goal_.target_pose.pose.position.x = loc->x;
    // goal_.target_pose.pose.position.y = loc->y;
    // goal_.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, loc->orientation);
    // client_.sendGoal(goal_);
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus GoToPose::onRunning()
{
    // actionlib::SimpleClientGoalState state = client_.getState();
    // if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
    //     ROS_DEBUG("[%s] reached goal", this->name().c_str());
    //     LOG_NAV_STOP(this->name());
    //     return BT::NodeStatus::SUCCESS;
    // }
    // else if (state == actionlib::SimpleClientGoalState::ACTIVE || state == actionlib::SimpleClientGoalState::PENDING)
    // {
    //     return BT::NodeStatus::RUNNING;
    // }
    // else
    // {
    //     LOG_NAV_STOP(this->name());
    //     ROS_ERROR("[%s] Failed to reach goal!", this->name().c_str());
    //     return BT::NodeStatus::FAILURE;
    // }
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 *
 */
void GoToPose::onHalted(){};

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList GoToPose::providedPorts()
{
    return {BT::InputPort<PathLocation>("loc")};
}

#pragma endregion

#pragma endregion

#pragma region SaveCurrentLocation

#pragma region public methods

/**
 * @brief Construct a new Save Current Location:: Save Current Location object
 *
 * @param name The name of the behavior
 * @param config The node configuration
 */
SaveCurrentLocation::SaveCurrentLocation(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config)
{
    ROS_LOG_INIT(this->name().c_str());
}

/**
 * @brief Sets the reference to the PathQueue
 *
 * @param path_queue Reference to the PathQueue
 */
void SaveCurrentLocation::init(PathQueue &path_queue)
{
    path_queue_ = &path_queue;
}

/**
 * @brief Sets the reference to the TomatoQueue
 *
 * @param tomato_queue Reference to the TomatoQueue
 */
void SaveCurrentLocation::init(TomatoQueue &tomato_queue)
{
    tomato_queue_ = &tomato_queue;
}

/**
 * @brief Works off the SaveCurrentLocation behavior
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus SaveCurrentLocation::tick()
{
    geometry_msgs::msg::PoseStamped current_location = ba_helper::GetCurrentPoseInMapFrame(BASE_FRAME);
    float rotation = ba_helper::ConvertQuaternionToAngle(current_location.pose.orientation);
    if (tomato_queue_->GetReachableTomatoes().size() > 0)
    {
        PathLocation current_location_as_struct = {(float)current_location.pose.position.x, (float)current_location.pose.position.y, rotation};
        path_queue_->Enqueue(QueueSide::front, current_location_as_struct);
    }
    else
    {
        path_queue_->Enqueue(QueueSide::front, path_queue_->Dequeue(QueueSide::front, false));
    }
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Ports to exchange information between each other.
 *
 * @return BT::PortsList
 */
BT::PortsList SaveCurrentLocation::providedPorts()
{
    return {BT::OutputPort<int>("num_locs")};
}

#pragma endregion

#pragma endregion

#pragma region WriteBasketChangeLocationToQueue

/**
 * @brief Construct a new Write Basket Change Location To Queue:: Write Basket Change Location To Queue object
 * 
 * @param name The name of the behavior
 * @param config The node configuration 
 */
WriteBasketChangeLocationToQueue::WriteBasketChangeLocationToQueue(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config)
{
    // ROS_LOG_INIT(this->name().c_str());
}

/**
 * @brief Works off the WriteBasketChangeLocationToQueue behavior
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus WriteBasketChangeLocationToQueue::tick()
{
    PathLocation basket_change_location = path_queue_->GetEmptyBasketLocation();
    path_queue_->Enqueue(QueueSide::front, basket_change_location);
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Sets the reference to the PathQueue
 *
 * @param path_queue Reference to the PathQueue
 */
void WriteBasketChangeLocationToQueue::init(PathQueue &path_queue)
{
    path_queue_ = &path_queue;
}

/**
 * @brief Ports to exchange information between each other.
 *
 * @return BT::PortsList
 */
BT::PortsList WriteBasketChangeLocationToQueue::providedPorts()
{
    return {};
}

#pragma endregion

#pragma region WriteChargingLocationToQueue

/**
 * @brief Construct a new Write Charging Location To Queue:: Write Charging Location To Queue object
 * 
 * @param name The name of the behavior
 * @param config The node configuration 
 */
WriteChargingLocationToQueue::WriteChargingLocationToQueue(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config)
{
    // ROS_LOG_INIT(this->name().c_str());
}

/**
 * @brief Works off the WriteChargingLocationToQueue behavior
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus WriteChargingLocationToQueue::tick()
{
    PathLocation charging_position = path_queue_->GetRechargeLocation();
    path_queue_->Enqueue(QueueSide::front, charging_position);
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Sets the reference to the PathQueue
 *
 * @param path_queue Reference to the PathQueue
 */
void WriteChargingLocationToQueue::init(PathQueue &path_queue)
{
    path_queue_ = &path_queue;
}

/**
 * @brief Ports to exchange information between each other.
 *
 * @return BT::PortsList
 */
BT::PortsList WriteChargingLocationToQueue::providedPorts()
{
    return {};
}

#pragma endregion
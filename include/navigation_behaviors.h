#pragma region includes

#include "behaviortree_cpp_v3/behavior_tree.h"
// #include <actionlib/client/simple_action_client.h>
//#include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp_components/register_node_macro.hpp"
// #include <move_base_msgs/MoveBaseAction.h>
#include "yaml-cpp/yaml.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "path_queue.h"
#include "ba_interfaces.h"
#include "manipulator.h"

// #include <tf/tf.h>

#include "ros_logs.h"

#include "helper.h"

#include "time_logger.h"
#ifdef TIME_LOGGER_ON
#define LOG_NAV_START(val) BATimeLogger::LogMoveBase(val, log_start)
#define LOG_NAV_STOP(val) BATimeLogger::LogMoveBase(val, log_stop)
#else
#define LOG_NAV_START(val)
#define LOG_NAV_STOP(val)
#endif

#pragma endregion

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#pragma region SetLocations

/**
 * @brief Class to set the path locations based on de locations.yaml
 *
 */
class SetLocations : public BT::SyncActionNode, public IBAInitPathQueue
{
public:
  SetLocations(const std::string &name, const BT::NodeConfiguration &config);
  void init(PathQueue &path_queue) override;
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

private:
  PathQueue *path_queue_;
};

#pragma endregion

#pragma region GetLocationFromQueue

/**
 * @brief Gets location from a queue of locations read from a list
 *
 */
class GetLocationFromQueue : public BT::SyncActionNode, public IBAInitPathQueue
{
public:
  GetLocationFromQueue(const std::string &name, const BT::NodeConfiguration &config);
  void init(PathQueue &path_queue) override;
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

private:
  std::deque<std::string> location_queue_;
  PathQueue *path_queue_;
};

#pragma endregion

#pragma region GoToPose

/**
 * @brief Go to a target location (wraps around `move_base`)
 *
 */
class GoToPose : public BT::StatefulActionNode, public IBAInitManipulatorNode
{
public:
  GoToPose(const std::string &name, const BT::NodeConfiguration &config);
  void init(Manipulator manipulator) override;
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts();

private:
  // MoveBaseClient client_;
  // move_base_msgs::MoveBaseGoal goal_;
  YAML::Node locations_;
  Manipulator manipulator_;
};

#pragma endregion

#pragma region SaveCurrentLocation

class SaveCurrentLocation : public BT::SyncActionNode, public IBAInitPathQueue, public IBAInitTomatoQueue
{
public:
  SaveCurrentLocation(const std::string &name, const BT::NodeConfiguration &config);
  BT::NodeStatus tick() override;
  void init(PathQueue &path_queue) override;
  void init(TomatoQueue &tomato_queue) override;
  static BT::PortsList providedPorts();

private:
  TomatoQueue *tomato_queue_;
  PathQueue *path_queue_;
};

#pragma endregion

#pragma region WriteBasketChangeLocationToQueue

/**
 * @brief Behavior to write the basket change location into the queue
 *
 */
class WriteBasketChangeLocationToQueue : public BT::SyncActionNode, public IBAInitPathQueue
{
public:
  WriteBasketChangeLocationToQueue(const std::string &name, const BT::NodeConfiguration &config);
  BT::NodeStatus tick() override;
  void init(PathQueue &path_queue) override;
  static BT::PortsList providedPorts();

private:
  PathQueue *path_queue_;
};

#pragma endregion

#pragma region WriteChargingLocationToQueue

/**
 * @brief Behavior to write the charging location into the queue
 *
 */
class WriteChargingLocationToQueue : public BT::SyncActionNode, public IBAInitPathQueue
{
public:
  WriteChargingLocationToQueue(const std::string &name, const BT::NodeConfiguration &config);
  BT::NodeStatus tick() override;
  void init(PathQueue &path_queue) override;
  static BT::PortsList providedPorts();

private:
  PathQueue *path_queue_;
};

#pragma endregion
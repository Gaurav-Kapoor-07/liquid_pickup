#pragma region includes

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/string.hpp"

#include "ba_frames_summit_xl.h"
#include "vector"
#include "string"

#pragma endregion

#pragma region SensorsDeploy

/**
 * @brief Receive sensor deploy poses at port
 *
 */
class SensorsDeploy : public BT::SyncActionNode
{
public:
  SensorsDeploy(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr deploy_sensor_names_publisher_;
};

#pragma endregion

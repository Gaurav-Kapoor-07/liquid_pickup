#ifndef ROS_LOGS_H
#define ROS_LOGS_H

#include <rclcpp/rclcpp.hpp>

#define ROS_LOG_INIT(name) RCLCPP_INFO(rclcpp::get_logger(#name), "[%s] Initialized!", name)

#endif
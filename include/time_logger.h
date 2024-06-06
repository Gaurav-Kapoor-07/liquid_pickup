#pragma once
#ifndef BA_TIME_LOGGER_H
#define BA_TIME_LOGGER_H

#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "ba_types.h"

// #define FILE_MOVEBASE   "move_base.csv"
#define FILE_MOVEARM    "move_group.csv"
#define FILE_PLANARM    "plan_move_group.csv"
// #define FILE_VISION     "image_processing.csv"
#define FILE_SCAN       "loc_scan.csv"
#define FILE_VALIDATE   "loc_validate.csv"

//#define TIME_LOGGER_ON

enum LogType{log_start, log_stop};
/**
 * @brief Class to log times for different parts of the behavîortree and the 
 * estimated poses of the tomatos found by the arm camera
 * 
 */
class BATimeLogger
{
    public:
    static void LogScan(geometry_msgs::msg::PoseStamped pose, float radius);
    static void LogValidate(geometry_msgs::msg::PoseStamped pose, float raduis);
    // static void LogMoveBase(std::string source, LogType type);
    static void LogMoveGroup(std::string source, LogType type);
    static void LogPlanArm(std::string source, LogType type);
    // static void LogImageProcessing(std::string source, LogType type);
    static void InitFiles(void);
    static void CloseFiles(void);
    // static std::ofstream file_movebase;
    static std::ofstream file_movegroup;
    static std::ofstream file_scan;
    static std::ofstream file_validate;
    // static std::ofstream file_imageprocessing;
    static void LogFile(std::string source, LogType type, std::ofstream &file);
};


#endif
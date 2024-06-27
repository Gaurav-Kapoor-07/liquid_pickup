#include "time_logger.h"

std::ofstream BATimeLogger::file_nav;
std::ofstream BATimeLogger::file_movegroup;
std::ofstream BATimeLogger::file_scan;
std::ofstream BATimeLogger::file_validate;


/**
 * @brief initializes all file descriptors, i.e. opens the file in append mode
 * 
 */
void BATimeLogger::InitFiles(void){
    file_nav.open(FILE_NAV,std::ios_base::app);
    file_movegroup.open(FILE_MOVEARM,std::ios_base::app);
    file_scan.open(FILE_SCAN,std::ios_base::app);
    file_validate.open(FILE_VALIDATE, std::ios_base::app);
    file_nav << "\n\n NEW RUN\n";
    file_movegroup << "\n\n NEW RUN\n";
}

/**
 * @brief closes all the log files
 * 
 */
void BATimeLogger::CloseFiles(void){
    file_nav.close();
    file_movegroup.close();
    file_scan.close();
    file_validate.close();
}

/**
 * @brief log the parameters to file loc_validate.csv
 * 
 * @param pose PoseStamped pose of the target in map_frame
 * @param radius radius of the target in m
 */
void BATimeLogger::LogValidate(geometry_msgs::msg::PoseStamped pose, double radius){
    file_validate<<pose.pose.position.x<<";"<<pose.pose.position.y<<";"<<pose.pose.position.z<<";"<<radius<<"\n";
}

/**
 * @brief log the parameters to file loc_scan.csv
 * 
 * @param pose PoseStamped pose of the target in map_frame
 * @param radius radius of the target in m
 */
void BATimeLogger::LogScan(geometry_msgs::msg::PoseStamped pose, double radius){
    file_scan<<pose.pose.position.x<<";"<<pose.pose.position.y<<";"<<pose.pose.position.z<<";"<<radius<<"\n";
}

/**
 * @brief log current time to file from a node specified in source
 * 
 * @param source node who loggs the value
 * @param type logtype start or stop
 * @param file destination file to log into
 */
void BATimeLogger::LogFile(std::string source, LogType type, std::ofstream &file){
    auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now().seconds();
    if(type == log_start){
        file<<source<<";"<<current_time<<";";
    }else{
        file<<current_time<<";\n";
    }
}

/**
 * @brief log time of Nav2 task
 * 
 * @param source name of the behavior; is logged in first column
 * @param type logtype start or stop
 */
void BATimeLogger::LogNav(std::string source, LogType type){
    LogFile(source, type, file_nav);
}

/**
 * @brief log time of movegroup task
 * 
 * @param source name of the behavior; is logged in first column
 * @param type logtype start or stop
 */
void BATimeLogger::LogMoveGroup(std::string source, LogType type){
    LogFile(source, type, file_movegroup);
}

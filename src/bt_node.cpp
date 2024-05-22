// Main behavior node for Summit-XL

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/bt_factory.h"
#include "loggers/zmq.hpp"

#define LOG_TIME

#ifdef LOG_TIME
#include "time_logger.h"
#endif

#include "navigation_behaviors.h"
// #include "vision_behaviors.h"
#include "manipulator_behaviors.h"
#include "gripper_behavior.h"

#include "manipulator.h"
#include "helper.h"
#include "basket.h"
#include "tomato_queue.h"
#include "path_queue.h"
#include "robot.h"
#include "ba_types.h"


#include <unistd.h>
#include <stdio.h>

#include "ba_interfaces.h"



using namespace BT;

int main(int argc, char **argv)
{
    // Initialize ROS node
    // ros::init(argc, argv, "bt_node");
    // ros::NodeHandle nh;
    // ros::AsyncSpinner async_spinner(1);
    // async_spinner.start();
    Manipulator manipulator;

    TomatoQueue tomato_queue_;
    PathQueue p_queue;

    // Build a behavior tree from XML and set it up for logging
    std::string behavior_tree_type;
    // ros::param::get("behavior_tree_type", behavior_tree_type);
    BT::Tree tree;

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<RobotInitializer>("RobotInitializer");
    factory.registerNodeType<GoToPose>("GoToPose");
    factory.registerNodeType<SetLocations>("SetLocations");
    factory.registerNodeType<GetLocationFromQueue>("GetLocationFromQueue");
    // factory.registerNodeType<LookForObject>("LookForObject");
    // factory.registerNodeType<ValidateTomato>("ValidateTomato");
    factory.registerNodeType<ManipulatorGraspTomato>("GraspTomato");
    factory.registerNodeType<DequeueTomato>("DequeueTomato");
    factory.registerNodeType<ManipulatorPregrasp>("pregraspTomato");
    factory.registerNodeType<ManipulatorDropTomato>("dropTomato");
    factory.registerNodeType<FilterTomatoQueue>("FilterTomatoQueue");
    factory.registerNodeType<ManipulatorPostgraspRetreat>("RetreatZ");
    factory.registerNodeType<ManipulatorScanPose>("ScanPose");
    factory.registerNodeType<BasketCheck>("BasketFull");
    factory.registerNodeType<BasketChange>("ChangeBasket");
    factory.registerNodeType<GripperActuator>("ChangeGripper");
    factory.registerNodeType<SaveCurrentLocation>("SaveCurrentLocation");
    factory.registerNodeType<WriteChargingLocationToQueue>("WriteChargingLocationToQueue");
    factory.registerNodeType<WriteBasketChangeLocationToQueue>("WriteBasketChangeLocationToQueue");
    factory.registerNodeType<BatteryCharge>("BatteryCharge");
    factory.registerNodeType<BatteryCheck>("BatteryCheck");
    try
    {
        std::string bt_xml;
        // ros::param::get("bt_xml", bt_xml);
        tree = factory.createTreeFromFile(bt_xml);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error creating BT from xml file!" << std::endl;
        std::cerr << e.what() << '\n';
    }

    // for (auto &node : tree.nodes)
    // {
    //     if (auto vis_node = dynamic_cast<IBAInitNodeHandle *>(node.get()))
    //     {
    //         // vis_node->init(nh);
    //     }
    //     if (auto vis_node = dynamic_cast<IBAInitManipulatorNode *>(node.get()))
    //     {
    //         vis_node->init(manipulator);
    //     }
    //     if (auto vis_node = dynamic_cast<IBAInitTomatoQueue *>(node.get()))
    //     {
    //         vis_node->init(tomato_queue_);
    //     }
    //     if (auto vis_node = dynamic_cast<IBAInitPathQueue *>(node.get()))
    //     {
    //         vis_node->init(p_queue);
    //     }
    // }

    unsigned max_msg_per_second = 25;
    unsigned publisher_port = 1666;
    unsigned server_port = 1667;
    // BT::PublisherZMQ publisher_zmq(tree, max_msg_per_second, publisher_port, server_port);

    // Tick the tree until it reaches a terminal state
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    // auto start = ros::Time::now().toSec();
    #ifdef LOG_TIME
    BATimeLogger::InitFiles();
    #endif
    while (status == BT::NodeStatus::RUNNING)
    {
        // status = tree.tickRoot();
        // ros::Duration(0.1).sleep();
    }

    // Output final results
    std::string status_str;
    if (status == BT::NodeStatus::SUCCESS)
    {
        status_str = "SUCCESS";
    }
    else
    {
        status_str = "FAILURE";
    }
    // auto stop = ros::Time::now().toSec();
    // auto seconds = stop-start;
    // ROS_INFO("Done with status %s!", status_str.c_str());
    // ROS_INFO("Used time: %.2lf", seconds);
    #ifdef LOG_TIME
    BATimeLogger::CloseFiles();
    #endif
    std::cout << '\n'
              << "Press a key to continue...";
    do
    {
    } while (std::cin.get() != '\n');

    return 0;
}

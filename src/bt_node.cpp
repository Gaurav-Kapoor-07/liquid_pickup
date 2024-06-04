// Main behavior node for Summit-XL

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger.h"
#include "behaviortree_cpp/bt_factory.h"
// #include "behaviortree_cpp/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>

#define LOG_TIME

#ifdef LOG_TIME
#include "time_logger.h"
#endif

// #include "navigation_behaviors.h"
// #include "vision_behaviors.h"
#include "manipulator_behaviors.h"  
#include "gripper_behavior.h"

#include "manipulator.h"
#include "helper.h"
// // #include "basket.h"
// // #include "tomato_queue.h"
// // #include "path_queue.h"
#include "robot.h"
#include "ba_types.h"


#include <unistd.h>
#include <stdio.h>

#include "ba_interfaces.h"

using namespace BT;
using namespace DummyNodes;

class LiquidPickup : public rclcpp::Node
{
  public:
    LiquidPickup()
    : Node("bt_node")
    {
      auto self = std::shared_ptr<LiquidPickup>(this, [](LiquidPickup*){});
      // ros::param::get("behavior_tree_type", behavior_tree_type);
      
      this->declare_parameter("behavior_tree_type", "behavior_tree_type");
      behavior_tree_type = this->get_parameter("behavior_tree_type").as_string();

      factory.registerNodeType<ApproachObject>("ApproachObject");
      factory.registerNodeType<SaySomething>("SaySomething");

      // factory.registerNodeType<RobotInitializer>("RobotInitializer");
      // factory.registerNodeType<ManipulatorGraspTomato>("GraspTomato");
      // factory.registerNodeType<DequeueTomato>("DequeueTomato");
      // factory.registerNodeType<ManipulatorPregrasp>("pregraspTomato");
      // factory.registerNodeType<ManipulatorDropTomato>("dropTomato");
      // factory.registerNodeType<FilterTomatoQueue>("FilterTomatoQueue");
      // factory.registerNodeType<ManipulatorPostgraspRetreat>("RetreatZ");
      // factory.registerNodeType<ManipulatorScanPose>("ScanPose");
      // factory.registerNodeType<BasketCheck>("BasketFull");
      // factory.registerNodeType<BasketChange>("ChangeBasket");
      // factory.registerNodeType<GripperActuator>("ChangeGripper");
      // factory.registerNodeType<SaveCurrentLocation>("SaveCurrentLocation");
      // factory.registerNodeType<WriteChargingLocationToQueue>("WriteChargingLocationToQueue");
      // factory.registerNodeType<WriteBasketChangeLocationToQueue>("WriteBasketChangeLocationToQueue");
      // factory.registerNodeType<BatteryCharge>("BatteryCharge");
      // factory.registerNodeType<BatteryCheck>("BatteryCheck");  

      try
      {
        this->declare_parameter("bt_xml", "test.xml");
        bt_xml = this->get_parameter("bt_xml").as_string(); 
        // ros::param::get("bt_xml", bt_xml);
        // factory.createTreeFromFile("/home/ros/rap/gaurav_ws/src/liquid_pickup/config/test.xml");

        std::string package_share_directory = ament_index_cpp::get_package_share_directory("liquid_pickup");

        // strcat(package_share_directory)
        // std::cerr << package_share_directory;

        std::string path_to_xml = package_share_directory + "/config/";

        // factory.createTreeFromFile("src/liquid_pickup/config/test.xml");

        tree = factory.createTreeFromFile(path_to_xml + bt_xml);
      }
      catch (const std::exception &e)
      {
        std::cerr << "Error creating BT from xml file!" << std::endl;
        std::cerr << e.what() << '\n';
      }

      // BT::Groot2Publisher publisher(tree, server_port);

      // auto node = tree.rootNode();

      // if (auto vis_node = dynamic_cast<IBAInitNodeHandle *>(node))
      // {
      //   vis_node->init(self);
      // }
      // if (auto vis_node = dynamic_cast<IBAInitManipulatorNode *>(node))
      // {
      //   vis_node->init(manipulator);
      // }
      // if (auto vis_node = dynamic_cast<IBAInitTomatoQueue *>(node.get()))
      // {
      //     vis_node->init(tomato_queue_);
      // }
      // if (auto vis_node = dynamic_cast<IBAInitPathQueue *>(node))
      // {
      //     // vis_node->init(p_queue);
      // }

      // BT::PublisherZMQ publisher_zmq(tree, max_msg_per_second, publisher_port, server_port);

      // // Tick the tree until it reaches a terminal state
      // BT::NodeStatus status = BT::NodeStatus::RUNNING;
      // auto start = this->now().seconds();
      
      // #ifdef LOG_TIME
      // BATimeLogger::InitFiles();
      // #endif
      // while (status == BT::NodeStatus::RUNNING)
      // {
      //   status = tree.tickRoot();
      //   // ros::Duration(0.1).sleep();
      //   rclcpp::sleep_for(std::chrono::nanoseconds(100000000));
      // }

      // // Output final results
      // std::string status_str;
      // if (status == BT::NodeStatus::SUCCESS)
      // {
      //     status_str = "SUCCESS";
      // }
      // else
      // {
      //     status_str = "FAILURE";
      // }
      // auto stop = this->now().seconds();;
      // auto seconds = stop-start;
      // // ROS_INFO("Done with status %s!", status_str.c_str());
      // // ROS_INFO("Used time: %.2lf", seconds);
      // RCLCPP_INFO(this->get_logger(), "Done with status %s!", status_str.c_str());
      // RCLCPP_INFO(this->get_logger(), "Used time: %.2lf", seconds);
      
      // #ifdef LOG_TIME
      // BATimeLogger::CloseFiles();
      // #endif
      // std::cout << '\n'
      //           << "Press a key to continue...";
      // do
      // {
      // } while (std::cin.get() != '\n');
    }

  private:
    // Manipulator manipulator;
    std::string behavior_tree_type;
    BT::Tree tree;
    BT::BehaviorTreeFactory factory;
    // BehaviorTreeFactory factory;
    std::string bt_xml;
    unsigned max_msg_per_second = 25;
    unsigned publisher_port = 1666;
    unsigned server_port = 1667;
};

int main(int argc, char *argv[])
{
    // Initialize ROS node
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr mnode = std::make_shared<LiquidPickup>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mnode);
    executor.spin();
    // ros::NodeHandle nh;
    // ros::AsyncSpinner async_spinner(1);
    // async_spinner.start();
    // Manipulator manipulator;

    // TomatoQueue tomato_queue_;
    // PathQueue p_queue;

    rclcpp::shutdown();

    return 0;
}

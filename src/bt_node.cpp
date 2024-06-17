// Main behavior node for Summit-XL

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>

#define LOG_TIME

#ifdef LOG_TIME
#include "time_logger.h"
#endif

#include "manipulator_behaviors.h"  
#include "gripper_behavior.h"

#include "manipulator.h"
#include "helper.h"
#include "robot.h"
#include "ba_types.h"

#include <unistd.h>
#include <stdio.h>

#include "ba_interfaces.h"

using namespace BT;

class LiquidPickup : public rclcpp::Node
{
  public:
    LiquidPickup()
    : Node("bt_node")
    {    
      this->declare_parameter("behavior_tree_type", "behavior_tree_type");
      behavior_tree_type = this->get_parameter("behavior_tree_type").as_string();

      factory.registerNodeType<RobotInitializer>("RobotInitializer");
      // factory.registerNodeType<ManipulatorGraspTomato>("GraspTomato");
      // factory.registerNodeType<ManipulatorPregrasp>("pregraspTomato");
      // factory.registerNodeType<ManipulatorDropTomato>("dropTomato");
      factory.registerNodeType<ManipulatorPostgraspRetreat>("RetreatZ");
      // factory.registerNodeType<ManipulatorScanPose>("ScanPose");
      // factory.registerNodeType<GripperActuator>("ChangeGripper");
      // factory.registerNodeType<BatteryCharge>("BatteryCharge");
      // factory.registerNodeType<BatteryCheck>("BatteryCheck");  

      std::string xml_models = BT::writeTreeNodesModelXML(factory);
      std::cerr << xml_models;

      try
      {
        // this->declare_parameter("bt_xml", "test.xml");
        this->declare_parameter("bt_xml", "test_2.xml");
        bt_xml = this->get_parameter("bt_xml").as_string(); 

        std::string package_share_directory = ament_index_cpp::get_package_share_directory("liquid_pickup");

        std::string path_to_xml = package_share_directory + "/config/";

        tree = factory.createTreeFromFile(path_to_xml + bt_xml);
      }
      catch (const std::exception &e)
      {
        std::cerr << "Error creating BT from xml file!" << std::endl;
        std::cerr << e.what() << '\n';
      }

      BT::Groot2Publisher publisher(tree, server_port);

      auto node = tree.rootNode();

      if (auto vis_node = dynamic_cast<IBAInitManipulatorNode *>(node))
      {
        vis_node->init(manipulator);
      }

      // Tick the tree until it reaches a terminal state
      BT::NodeStatus status = BT::NodeStatus::RUNNING;
      auto start = this->now().seconds();
      
      #ifdef LOG_TIME
      BATimeLogger::InitFiles();
      #endif

      status = tree.tickWhileRunning(std::chrono::milliseconds(100)); 

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
      auto stop = this->now().seconds();
      auto seconds = stop - start;

      RCLCPP_INFO(this->get_logger(), "Done with status %s!", status_str.c_str());
      RCLCPP_INFO(this->get_logger(), "Used time: %.2lf seconds", seconds);
      
      #ifdef LOG_TIME
      BATimeLogger::CloseFiles();
      #endif
      std::cerr << '\n'
                << "Press a key to continue...";
      do
      {
      } while (std::cin.get() != '\n');
    }

  private:
    Manipulator manipulator;
    std::string behavior_tree_type;
    BT::Tree tree;
    BT::BehaviorTreeFactory factory;
    std::string bt_xml;
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
    // rclcpp::spin(std::make_shared<LiquidPickup>());

    rclcpp::shutdown();

    return 0;
}

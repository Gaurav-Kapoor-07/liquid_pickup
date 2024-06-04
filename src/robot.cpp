// #include "robot.h"

// #pragma region RobotInitializer

// #pragma region public methods

// /**
//  * @brief Construct a new Robot Initializer:: Robot Initializer object
//  *
//  * @param name The name of the behavior
//  * @param config The node configuration
//  */
// RobotInitializer::RobotInitializer(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config)
// {
//     ROS_LOG_INIT(this->name().c_str());
// }

// /**
//  * @brief Set the Manipulator object
//  * 
//  * @param manipulator The manipulator 
//  */
// void RobotInitializer::init(Manipulator manipulator)
// {
//     manipulator_ = manipulator;
// }

// /**
//  * @brief Set the NodeHandle reference object
//  * 
//  * @param node_handle Reference to the NodeHandle 
//  */
// void RobotInitializer::init(std::shared_ptr<rclcpp::Node> node_handle)
// {
//     node_handle_ = node_handle;
// }

// /**
//  * @brief Handles the tick from the behavior tree
//  *
//  * @return BT::NodeStatus The status of the node
//  */
// BT::NodeStatus RobotInitializer::tick()
// {
//     // moveit::core::MoveItErrorCode code = SetInitialPosition();
//     // LaunchBasket();
//     return BT::NodeStatus::SUCCESS;
// }

// /**
//  * @brief Gets the ports provided by this behavior.
//  *
//  * @return BT::PortsList The list of the ports
//  */
// BT::PortsList RobotInitializer::providedPorts()
// {
//     return {BT::InputPort<std::string>("message")};
// }

// #pragma endregion

// #pragma region private methods

// /**
//  * @brief Sets the initial position of the robot
//  *
//  * @return moveit::core::MoveItErrorCode The error code
//  */
// moveit::core::MoveItErrorCode RobotInitializer::SetInitialPosition()
// {
//     moveit::core::MoveItErrorCode code = manipulator_.MoveToInitialPosition();
//     return code;
// }

// /**
//  * @brief Launches the basket into gazebo and attaches it to the summit_xl robot
//  *
//  */
// // void RobotInitializer::LaunchBasket()
// // {
// //     // ROS_INFO("Service '/gazebo/spawn_urdf_model' is about to get called.");
// //     // bool connected = ros::service::waitForService("/gazebo/spawn_urdf_model", 10);

// //     // ROS_INFO("Service received");
// //     char modelname[20];
// //     char linkname[30];
// //     std::srand(std::time(NULL));
// //     int model_id = std::rand() % 1000;
// //     sprintf(modelname, "basket_%d", model_id);
// //     sprintf(linkname, "tomato_basket_link_%d", model_id);
// //     // ros::ServiceClient spawn_model_client = node_handle_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
// //     // gazebo_msgs::SpawnModel spawn_model;
// //     // spawn_model.request.model_name = modelname;

// //     // generate urdf-file from xacro
// //     // std::string ba22_toff_06_path = ros::package::getPath("ba22_toff_06");
// //     // std::string path_to_xacro = ba22_toff_06_path + "/urdf/basket/basket.urdf.xacro";
// //     // std::string path_to_urdf = ba22_toff_06_path + "/urdf/basket/basket.urdf";
// //     // std::string xacro_cmd = "xacro " + path_to_xacro + " > " + path_to_urdf + " basket_link_id:=" + linkname;

// //     // std::system(xacro_cmd.c_str());

// //     // load urdf file
// //     // std::string urdf_filename = path_to_urdf;
// //     // ROS_INFO("loading file: %s", urdf_filename.c_str());
// //     // read urdf / gazebo model xml from file
// //     // TiXmlDocument xml_in(urdf_filename);
// //     // xml_in.LoadFile();
// //     // std::ostringstream stream;
// //     // stream << xml_in;
// //     // spawn_model.request.model_xml = stream.str(); // load xml file

// //     // spawn_model.request.robot_namespace = "summit_xl";
// //     // geometry_msgs::Pose pose;
// //     // spawn_model.request.reference_frame = "summit_xl_base_footprint";
// //     // pose.position.x = -0.52;
// //     // pose.position.y = 0;
// //     // pose.position.z = 0.38;

// //     // pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

// //     // spawn_model.request.initial_pose = pose;

// //     // bool spawned = spawn_model_client.call(spawn_model);

// //     // // ROS_INFO("Service '/link_attacher_node/attach' is about to get called.");
// //     // char system_call[200];
// //     // sprintf(system_call, "rosservice call /link_attacher_node/attach \"model_name_1: 'summit_xl'\nlink_name_1: 'summit_xl_base_footprint'\nmodel_name_2: '%s'\nlink_name_2: '%s'\" ", modelname, linkname);
// //     // std::system(system_call);

// //     // ROS_INFO("Basket successfully attached!");
// // }

// #pragma endregion

// #pragma endregion

// #pragma region BatteryCheck

// /**
//  * @brief Construct a new Battery Check:: Battery Check object
//  *
//  * @param name The name of the behavior
//  * @param config The node configuration
//  */
// BatteryCheck::BatteryCheck(const std::string &name, const BT::NodeConfiguration &config) : BT::ConditionNode(name, config)
// {
//     ROS_LOG_INIT(this->name().c_str());
//     battery_empty_ = false;
//     this->node_handle_->declare_parameter<float>("battery_timer", timer_duration_);
//     // ros::param::get("battery_timer", timer_duration_);
//     // timer_ = node_handle_.createTimer(ros::Duration(timer_duration_), &BatteryCheck::TimerCallback, this);

//     int64_t nanoseconds = static_cast<int64_t>(timer_duration_ * 1e9);

//     timer_ = this->node_handle_->create_wall_timer(std::chrono::nanoseconds(nanoseconds), std::bind(&BatteryCheck::TimerCallback, this));
//     start_ = rclcpp::Clock{RCL_ROS_TIME}.now().seconds();
//     // start_ = ros::Time::now().toSec();
// }

// /**
//  * @brief Initializes the NodeHandle reference
//  *
//  * @param node_handle The NodeHandle reference
//  */
// void BatteryCheck::init(std::shared_ptr<rclcpp::Node> node_handle)
// {
//     node_handle_ = node_handle;
// }

// /**
//  * @brief Handles the tick from the behavior tree
//  *
//  * @return BT::NodeStatus The status of the node
//  */
// BT::NodeStatus BatteryCheck::tick()
// {
//     // double now = ros::Time::now().toSec();
//     double now = rclcpp::Clock{RCL_ROS_TIME}.now().seconds();

//     double current_battery_level = (90 - (now - start_) / timer_duration_ * 90) + 10;

//     if (battery_empty_)
//     {
//         battery_empty_ = false;
//         // start_ = ros::Time::now().toSec();
//         start_ = rclcpp::Clock{RCL_ROS_TIME}.now().seconds();
//         if (current_battery_level < 3)
//         {
//             // ROS_WARN("[MOCKED BEHAVIOR] Battery capacity too low: <3 %%!");
//             RCLCPP_WARN(this->node_handle_->get_logger(), "[MOCKED BEHAVIOR] Battery capacity too low: <3 %%!");
//             return BT::NodeStatus::FAILURE;
//         }
//         // ROS_WARN("[MOCKED BEHAVIOR] Battery capacity too low: %.2lf %%!", current_battery_level);
//         RCLCPP_WARN(this->node_handle_->get_logger(), "[MOCKED BEHAVIOR] Battery capacity too low: %.2lf %%!", current_battery_level);
//         return BT::NodeStatus::FAILURE;
//     }
//     else
//     {
//         // ROS_INFO("[MOCKED BEHAVIOR] Battery capacity ok: %.2lf %%!", current_battery_level);
//         RCLCPP_INFO(this->node_handle_->get_logger(), "[MOCKED BEHAVIOR] Battery capacity ok: %.2lf %%!", current_battery_level);
//         return BT::NodeStatus::SUCCESS;
//     }
// }

// /**
//  * @brief Ports to exchange information between each other.
//  *
//  * @return BT::PortsList
//  */
// BT::PortsList BatteryCheck::providedPorts()
// {
//     return {};
// }

// // void BatteryCheck::TimerCallback(const ros::TimerEvent &timer_event)
// // {
// //     battery_empty_ = true;
// //     ROS_WARN("Battery capacity from now on too low!");
// // }

// void BatteryCheck::TimerCallback()
// {
//     battery_empty_ = true;
//     RCLCPP_WARN(this->node_handle_->get_logger(), "Battery capacity from now on too low!");
//     // ROS_WARN("Battery capacity from now on too low!");
// }

// #pragma endregion

// #pragma region BatteryCharge

// /**
//  * @brief Construct a new Battery Check:: Battery Check object
//  *
//  * @param name The name of the behavior
//  * @param config The node configuration
//  */
// BatteryCharge::BatteryCharge(const std::string &name, const BT::NodeConfiguration &config) : BT::ConditionNode(name, config)
// {
//     ROS_LOG_INIT(this->name().c_str());
// }

// /**
//  * @brief Handles the tick from the behavior tree
//  *
//  * @return BT::NodeStatus The status of the node
//  */
// BT::NodeStatus BatteryCharge::tick()
// {
//     // ROS_WARN("Mocking behavior of charging:");
//     RCLCPP_WARN(this->node_handle_2->get_logger(), "Mocking behavior of charging:");

//     // for (int i = 0; i <= 10; i++)
//     // {
//     //     ros::Duration(0.5).sleep();
//     //     ROS_INFO("Battery capacity: %d %%", i * 10);
//     // }

//     for (int i = 0; i <= 10; i++)
//     {
//         // ros::Duration(0.5).sleep();
//         rclcpp::sleep_for(std::chrono::nanoseconds(500000000));
//         RCLCPP_INFO(this->node_handle_2->get_logger(), "Battery capacity: %d %%", i * 10);;
//         // ROS_INFO("Battery capacity: %d %%", i * 10);
//     }

//     return BT::NodeStatus::SUCCESS;
// }

// /**
//  * @brief Ports to exchange information between each other.
//  *
//  * @return BT::PortsList
//  */
// BT::PortsList BatteryCharge::providedPorts()
// {
//     return {};
// }

// #pragma endregion

#include "robot.h"

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
// BT_REGISTER_NODES(factory)
// {
//     DummyNodes::RegisterNodes(factory);
// }

namespace DummyNodes
{

// BT::NodeStatus CheckBattery()
// {
//     std::cout << "[ Battery: OK ]" << std::endl;
//     return BT::NodeStatus::SUCCESS;
// }

// BT::NodeStatus CheckTemperature()
// {
//     std::cout << "[ Temperature: OK ]" << std::endl;
//     return BT::NodeStatus::SUCCESS;
// }

// BT::NodeStatus SayHello()
// {
//     std::cout << "Robot says: Hello World" << std::endl;
//     return BT::NodeStatus::SUCCESS;
// }

// BT::NodeStatus GripperInterface::open()
// {
//     _opened = true;
//     std::cout << "GripperInterface::open" << std::endl;
//     return BT::NodeStatus::SUCCESS;
// }

// BT::NodeStatus GripperInterface::close()
// {
//     std::cout << "GripperInterface::close" << std::endl;
//     _opened = false;
//     return BT::NodeStatus::SUCCESS;
// }

BT::NodeStatus ApproachObject::tick()
{
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SaySomething::tick()
{
    auto msg = getInput<std::string>("message");
    if (!msg)
    {
        throw BT::RuntimeError( "missing required input [message]: ", msg.error() );
    }

    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// BT::NodeStatus SaySomethingSimple(BT::TreeNode &self)
// {
//     auto msg = self.getInput<std::string>("message");
//     if (!msg)
//     {
//         throw BT::RuntimeError( "missing required input [message]: ", msg.error() );
//     }

//     std::cout << "Robot says: " << msg.value() << std::endl;
//     return BT::NodeStatus::SUCCESS;
// }

}


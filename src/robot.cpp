#include "robot.h"

#pragma region RobotInitializer

#pragma region public methods

/**
 * @brief Construct a new Robot Initializer:: Robot Initializer object
 *
 * @param name The name of the behavior
 * @param config The node configuration
 */
RobotInitializer::RobotInitializer(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node): BT::SyncActionNode(name, config), manipulator_(node)
{
    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s] Node shared pointer was passed!", this->name().c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] Initialized!", this->name().c_str());
}

/**
 * @brief Handles the tick from the behavior tree
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus RobotInitializer::tick()
{
    moveit::core::MoveItErrorCode code = SetInitialPosition();
    // LaunchBasket();
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList RobotInitializer::providedPorts()
{
    return {BT::InputPort<std::string>("message")};
}

#pragma endregion

#pragma region private methods

/**
 * @brief Sets the initial position of the robot
 *
 * @return moveit::core::MoveItErrorCode The error code
 */
moveit::core::MoveItErrorCode RobotInitializer::SetInitialPosition()
{
    moveit::core::MoveItErrorCode code = manipulator_.MoveToInitialPosition();
    return code;
}

/**
 * @brief Launches the basket into gazebo and attaches it to the summit_xl robot
 *
 */
void RobotInitializer::LaunchBasket()
{
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client = node_->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "waiting for service to appear...");
    }

    auto spawn_entity_request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();

    char modelname[20];
    char linkname[30];
    std::srand(std::time(NULL));
    int model_id = std::rand() % 1000;
    sprintf(modelname, "basket_%d", model_id);
    sprintf(linkname, "tomato_basket_link_%d", model_id);   

    // generate urdf-file from xacro
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("liquid_pickup");
    std::string path_to_xacro = package_share_directory + "/urdf/basket/basket.urdf.xacro";
    std::string path_to_urdf = package_share_directory + "/urdf/basket/basket.urdf";
    std::string xacro_cmd = "xacro " + path_to_xacro + " > " + path_to_urdf + " basket_link_id:=" + linkname;

    std::system(xacro_cmd.c_str());

    // load urdf file
    std::string urdf_filename = path_to_urdf;
    RCLCPP_INFO(node_->get_logger(), "loading file: %s", urdf_filename.c_str());

    // read urdf / gazebo model xml from file
    TiXmlDocument xml_in(urdf_filename);
    xml_in.LoadFile();
    std::ostringstream stream;
    stream << xml_in;
    spawn_entity_request->xml = stream.str(); // load xml file
    
    spawn_entity_request->name = "swab_container";
    spawn_entity_request->robot_namespace = "/summit";
    
    geometry_msgs::msg::Pose pose;

    pose.position.x = -0.52;
    pose.position.y = 0.0;
    pose.position.z = 0.38;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    spawn_entity_request->initial_pose = pose;

    spawn_entity_request->reference_frame = "base_footprint";

    auto result_future = client->async_send_request(spawn_entity_request);

    auto result = result_future.get();

    RCLCPP_INFO(node_->get_logger(), "result status message: %s, success (0: False, 1: True)? = %d", result->status_message.c_str(), result->success);
}

#pragma endregion

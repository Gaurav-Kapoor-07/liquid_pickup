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
    LaunchSwabContainer();
    LaunchSwab();
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList RobotInitializer::providedPorts()
{
    return {BT::InputPort<double>("swab_x"), BT::InputPort<double>("swab_y"), BT::InputPort<double>("swab_z"), BT::InputPort<double>("swab_yaw"), BT::InputPort<double>("swab_container_x"), BT::InputPort<double>("swab_container_y"), BT::InputPort<double>("swab_container_z"), BT::InputPort<double>("swab_container_yaw")};
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
 * @brief Launches the Swab Container into gazebo and attaches it to the summit_xl robot
 *
 */
void RobotInitializer::LaunchSwabContainer()
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
    sprintf(modelname, "swab_container_%d", model_id);
    sprintf(linkname, "swab_container_link_%d", model_id);   

    // generate urdf-file from xacro
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("liquid_pickup");
    std::string path_to_xacro = package_share_directory + "/urdf/swab_container/swab_container.urdf.xacro";
    std::string path_to_urdf = package_share_directory + "/urdf/swab_container/swab_container.urdf";
    std::string xacro_cmd = "xacro " + path_to_xacro + " > " + path_to_urdf + " swab_container_link_id:=" + linkname;

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
    
    BT::Optional<double> swab_container_x_ = getInput<double>("swab_container_x").value();
    BT::Optional<double> swab_container_y_ = getInput<double>("swab_container_y").value();
    BT::Optional<double> swab_container_z_ = getInput<double>("swab_container_z").value();
    BT::Optional<double> swab_container_yaw_ = getInput<double>("swab_container_yaw").value();
    
    geometry_msgs::msg::Pose pose;

    pose.position.x = swab_container_x_.value();
    pose.position.y = swab_container_y_.value();
    pose.position.z = swab_container_z_.value();
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(swab_container_yaw_.value() / 2.0);
    pose.orientation.w = std::cos(swab_container_yaw_.value() / 2.0);

    spawn_entity_request->initial_pose = pose;

    spawn_entity_request->reference_frame = BASE_FRAME;

    auto result_future = client->async_send_request(spawn_entity_request);

    auto result = result_future.get();

    RCLCPP_INFO(node_->get_logger(), "result status message: %s, success (0: False, 1: True)? = %d", result->status_message.c_str(), result->success);
}

/**
 * @brief Launches the Swab into gazebo and attaches it to the summit_xl robot
 *
 */
void RobotInitializer::LaunchSwab()
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

    // generate urdf-file from xacro
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("liquid_pickup");
    std::string path_to_urdf = package_share_directory + "/urdf/swab/model.sdf";

    // load urdf file
    std::string urdf_filename = path_to_urdf;
    RCLCPP_INFO(node_->get_logger(), "loading file: %s", urdf_filename.c_str());

    // read urdf / gazebo model xml from file
    TiXmlDocument xml_in(urdf_filename);
    xml_in.LoadFile();
    std::ostringstream stream;
    stream << xml_in;
    spawn_entity_request->xml = stream.str(); // load xml file
    
    spawn_entity_request->name = "swab";
    spawn_entity_request->robot_namespace = "/summit";
    
    BT::Optional<double> swab_x_ = getInput<double>("swab_x").value();
    BT::Optional<double> swab_y_ = getInput<double>("swab_y").value();
    BT::Optional<double> swab_z_ = getInput<double>("swab_z").value();
    BT::Optional<double> swab_yaw_ = getInput<double>("swab_yaw").value();
    
    geometry_msgs::msg::Pose pose;

    pose.position.x = swab_x_.value();
    pose.position.y = swab_y_.value();
    pose.position.z = swab_z_.value();
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(swab_yaw_.value() / 2.0);
    pose.orientation.w = std::cos(swab_yaw_.value() / 2.0);

    spawn_entity_request->initial_pose = pose;

    spawn_entity_request->reference_frame = BASE_FRAME;

    auto result_future = client->async_send_request(spawn_entity_request);

    auto result = result_future.get();

    RCLCPP_INFO(node_->get_logger(), "result status message: %s, success (0: False, 1: True)? = %d", result->status_message.c_str(), result->success);
}

#pragma endregion

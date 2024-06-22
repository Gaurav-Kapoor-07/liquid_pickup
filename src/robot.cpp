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
// void RobotInitializer::LaunchBasket()
// {
    // ROS_INFO("Service '/gazebo/spawn_urdf_model' is about to get called.");
    // bool connected = ros::service::waitForService("/gazebo/spawn_urdf_model", 10);

    // ROS_INFO("Service received");
    // char modelname[20];
    // char linkname[30];
    // std::srand(std::time(NULL));
    // int model_id = std::rand() % 1000;
    // sprintf(modelname, "basket_%d", model_id);
    // sprintf(linkname, "target_basket_link_%d", model_id);
    // ros::ServiceClient spawn_model_client = node_handle_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    // gazebo_msgs::SpawnModel spawn_model;
    // spawn_model.request.model_name = modelname;

    // generate urdf-file from xacro
    // std::string ba22_toff_06_path = ros::package::getPath("ba22_toff_06");
    // std::string path_to_xacro = ba22_toff_06_path + "/urdf/basket/basket.urdf.xacro";
    // std::string path_to_urdf = ba22_toff_06_path + "/urdf/basket/basket.urdf";
    // std::string xacro_cmd = "xacro " + path_to_xacro + " > " + path_to_urdf + " basket_link_id:=" + linkname;

    // std::system(xacro_cmd.c_str());

    // load urdf file
    // std::string urdf_filename = path_to_urdf;
    // ROS_INFO("loading file: %s", urdf_filename.c_str());
    // read urdf / gazebo model xml from file
    // TiXmlDocument xml_in(urdf_filename);
    // xml_in.LoadFile();
    // std::ostringstream stream;
    // stream << xml_in;
    // spawn_model.request.model_xml = stream.str(); // load xml file

    // spawn_model.request.robot_namespace = "summit_xl";
    // geometry_msgs::Pose pose;
    // spawn_model.request.reference_frame = "summit_xl_base_footprint";
    // pose.position.x = -0.52;
    // pose.position.y = 0;
    // pose.position.z = 0.38;

    // pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

    // spawn_model.request.initial_pose = pose;

    // bool spawned = spawn_model_client.call(spawn_model);

    // // ROS_INFO("Service '/link_attacher_node/attach' is about to get called.");
    // char system_call[200];
    // sprintf(system_call, "rosservice call /link_attacher_node/attach \"model_name_1: 'summit_xl'\nlink_name_1: 'summit_xl_base_footprint'\nmodel_name_2: '%s'\nlink_name_2: '%s'\" ", modelname, linkname);
    // std::system(system_call);

    // ROS_INFO("Basket successfully attached!");
// }

#pragma endregion

#include "move_it_task_constructor_behaviors.h"

#pragma region MoveItTaskConstructor

/**
 * @brief Construct a new MoveItTaskConstructor :: MoveItTaskConstructor object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */

MoveItTaskConstructor::MoveItTaskConstructor(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config)
{
    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s] Node shared pointer was passed!", this->name().c_str());
    }

    // moveit::planning_interface::MoveGroupInterface::Options manipulator_options_(GROUP_NAME, ROBOT_DESCRIPTION, "/summit");

    // manipulator_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, manipulator_options_);

    RCLCPP_INFO(node_->get_logger(), "[%s] Initialized!", this->name().c_str());
}

mtc::Task MoveItTaskConstructor::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_, ROBOT_DESCRIPTION);

  // Set task properties§
  task.setProperty("group", GROUP_NAME);
  task.setProperty("eef", "arm_tool0");
  task.setProperty("ik_frame", "arm_tool0");

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

//   auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

//   auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
//   cartesian_planner->setMaxVelocityScalingFactor(1.0);
//   cartesian_planner->setMaxAccelerationScalingFactor(1.0);
//   cartesian_planner->setStepSize(.01);

  auto stage_go_to_pose =
      std::make_unique<mtc::stages::MoveTo>("go to pose", interpolation_planner);
  stage_go_to_pose->setGroup(GROUP_NAME);
  stage_go_to_pose->setGoal("home");
  stage_go_to_pose->setIKFrame("arm_tool0");
  stage_go_to_pose->setTimeout(5.0);

  task.add(std::move(stage_go_to_pose));

  return task;
}

void MoveItTaskConstructor::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    std::stringstream rclcpp_stream_e;
    rclcpp_stream_e << e;

    RCLCPP_ERROR(node_->get_logger(), rclcpp_stream_e.str().c_str());
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR(node_->get_logger(), "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Task execution failed");
    return;
  }

  return;
}

void MoveItTaskConstructor::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  //   object.header.frame_id = "world";
  object.header.frame_id = BASE_FRAME;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 2.0;
  pose.position.y = -0.25;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus MoveItTaskConstructor::onStart()
{
    // setupPlanningScene();
    doTask();

    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus MoveItTaskConstructor::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void MoveItTaskConstructor::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList MoveItTaskConstructor::providedPorts()
{
    return {BT::InputPort<std::string>("action")};
}

#pragma endregion

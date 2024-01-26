#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#define SUCCESS moveit::core::MoveItErrorCode::SUCCESS

static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_TOOL = "gripper";

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place_node");

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  // define node options for moveit2 parameters overrides
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pick_and_place_node", options);

  // add node to executor and spin
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // move group interface
  auto move_group_arm =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          node, PLANNING_GROUP_ARM);
  auto move_group_tool =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          node, PLANNING_GROUP_TOOL);

  auto joints_arm =
      move_group_arm->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  auto joints_tool = move_group_tool->getCurrentState()->getJointModelGroup(
      PLANNING_GROUP_TOOL);

  // get current state of system
  std::vector<double> positions_arm;
  std::vector<double> positions_tool;

  moveit::core::RobotStatePtr state_arm = move_group_arm->getCurrentState(10);
  moveit::core::RobotStatePtr state_tool = move_group_tool->getCurrentState(10);

  state_arm->copyJointGroupPositions(joints_arm, positions_arm);
  state_tool->copyJointGroupPositions(joints_tool, positions_tool);

  // define planning interface
  moveit::planning_interface::MoveGroupInterface::Plan plan_arm;
  moveit::planning_interface::MoveGroupInterface::Plan plan_tool;

  const double eef_step = 0.01;
  const double jump_threshold = 0.0;

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  // start robot movement
  RCLCPP_INFO(LOGGER, "Initilization done starting robot arm movement!");
  move_group_arm->setStartStateToCurrentState();
  move_group_tool->setStartStateToCurrentState();

  // position : home
  move_group_arm->setNamedTarget("home");
  if (move_group_arm->plan(plan_arm) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `home`");
    move_group_arm->execute(plan_arm);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `home`");
  }

  // position : grasp
  move_group_arm->setNamedTarget("grasp");
  if (move_group_arm->plan(plan_arm) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `grasp`");
    move_group_arm->execute(plan_arm);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `grasp`");
  }

  // tool : open
  move_group_tool->setNamedTarget("gripper_open");
  if (move_group_tool->plan(plan_tool) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `gripper_open`");
    move_group_tool->execute(plan_tool);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `gripper_open`");
  }

  // define approach target_pose message
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.34;
  target_pose.position.y = -0.02;
  target_pose.position.z = 0.20;
  target_pose.orientation.x = -1.0;
  target_pose.orientation.y = 0.00;
  target_pose.orientation.z = 0.00;
  target_pose.orientation.w = 0.00;

  // define waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints_approach;
  waypoints_approach.push_back(target_pose);
  target_pose.position.z -= 0.02;
  waypoints_approach.push_back(target_pose);

  // compute trajectory
  auto f = move_group_arm->computeCartesianPath(
      waypoints_approach, eef_step, jump_threshold, trajectory_approach);

  // position : approach (using IK)
  move_group_tool->execute(trajectory_approach);
  RCLCPP_INFO(LOGGER, "Plan passed : `approach`");

  // tool : close
  move_group_tool->setNamedTarget("gripper_close");
  if (move_group_tool->plan(plan_tool) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `gripper_close`");
    move_group_tool->execute(plan_tool);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `gripper_close`");
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  move_group_tool->setNamedTarget("gripper_hold");
  if (move_group_tool->plan(plan_tool) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `gripper_hold`");
    move_group_tool->execute(plan_tool);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `gripper_hold`");
  }

  // define waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints_retreat;
  target_pose.position.z += 0.02;
  waypoints_retreat.push_back(target_pose);

  // compute trajectory
  auto b = move_group_arm->computeCartesianPath(
      waypoints_retreat, eef_step, jump_threshold, trajectory_retreat);

  // position : retreat (using IK)
  move_group_tool->execute(trajectory_retreat);
  RCLCPP_INFO(LOGGER, "Plan passed : `retreat`");

  // position : rotate
  move_group_arm->setNamedTarget("rotate");
  if (move_group_arm->plan(plan_arm) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `rotate`");
    move_group_arm->execute(plan_arm);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `rotate`");
  }

  // tool : open
  move_group_tool->setNamedTarget("gripper_open");
  if (move_group_tool->plan(plan_tool) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `gripper_open`");
    move_group_tool->execute(plan_tool);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `gripper_open`");
  }

  // position : stand
  move_group_arm->setNamedTarget("stand");
  if (move_group_arm->plan(plan_arm) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `stand`");
    move_group_arm->execute(plan_arm);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `stand`");
  }

  // suppress unused variables warnings
  (void)f;
  (void)b;

  // shutdown
  rclcpp::shutdown();
  return 0;
}
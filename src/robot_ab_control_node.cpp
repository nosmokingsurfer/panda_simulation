#include <ros/ros.h>
#include <string>

#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

static const std::string PLANNING_GROUP = "panda_arm";

int main(int argc, char** argv)
{
  ROS_INFO("RUNNING robot_ab_control_node");
  ros::init(argc, argv, "robot_ab_control_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;

  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  for (int i = 0; i < 5; i++)
  {
    visual_tools.prompt("Press next to start task_1");
    ROS_INFO("Step number %i", i);

    // Picking up new random state in joint space
    robot_state::RobotStatePtr new_state(new robot_state::RobotState(*move_group.getCurrentState()));
    new_state->setToRandomPositions(joint_model_group);

    Eigen::Isometry3d end_effector = new_state->getGlobalLinkTransform("panda_link8");
    ROS_INFO("Next point to plan to : %s", end_effector);

    geometry_msgs::PoseStamped trg_pose;
    tf::poseEigenToMsg(end_effector, trg_pose.pose);

    move_group.setPoseTarget(trg_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Planning state = %s", success ? "" : "FAILED");

    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    move_group.execute(my_plan);
  }

  ros::waitForShutdown();
  return 0;
}
#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/PlanningScene.h>

#include <control_msgs/FollowJointTrajectoryActionFeedback.h>

#include <fstream>

#include <boost/bind.hpp>

static const std::string PLANNING_GROUP = "panda_arm";

void feedbackCallBack(const boost::shared_ptr<control_msgs::FollowJointTrajectoryActionFeedback const> msg,
                      moveit::planning_interface::MoveGroupInterface &group)
{
  // opening csv file to store the data
  std::ofstream output;
  output.open("../../../src/panda_simulation/error.csv", std::ios::app);

  // points from feeedback message from gazebo
  auto &actual = msg->feedback.actual;
  auto &desired = msg->feedback.desired;

  // getting kinematic model for forward kinematics
  auto kinematic_model = group.getRobotModel();
  robot_state::RobotState state(kinematic_model);

  // writing desired pose of the end effector
  state.setJointGroupPositions(kinematic_model->getJointModelGroup("panda_arm"), desired.positions);
  Eigen::Affine3d eff = state.getGlobalLinkTransform("panda_link8");
  output << msg->feedback.header.seq << " " << eff.translation()[0] << " " << eff.translation()[1] << " "
         << eff.translation()[2] << " ";

  // writing actual pose of the end effector
  state.setJointGroupPositions(kinematic_model->getJointModelGroup("panda_arm"), actual.positions);
  eff = state.getGlobalLinkTransform("panda_link8");
  output << eff.translation()[0] << " " << eff.translation()[1] << " " << eff.translation()[2] << " ";

  // writing desired joint positions
  for (const auto &j : desired.positions)
  {
    output << j << " ";
  }

  // writing actual joint positions
  for (const auto &j : actual.positions)
  {
    output << j << " ";
  }

  output << "\n";

  output.close();
}

int main(int argc, char **argv)
{
  ROS_INFO("RUNNING robot_ab_control_node");
  ros::init(argc, argv, "robot_ab_control_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;

  // creating visual tools instance to be able control test from rviz motion planning tab
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // creating move group for panda manipulator
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  
  // creating move group planning scene publisher
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene",1);
  ros::WallDuration sleep_t(0.5);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }
  moveit_msgs::PlanningScene planning_scene;

  // adding collision object
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "world";
  collision_object.id = "ground";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 10.0;
  primitive.dimensions[1] = 10.0;
  primitive.dimensions[2] = 1.0;

  geometry_msgs::Pose floor_pose;
  floor_pose.orientation.w = 1;
  floor_pose.orientation.x = 0;
  floor_pose.orientation.y = 0;
  floor_pose.orientation.z = 0;

  floor_pose.position.x = 0;
  floor_pose.position.y = 0;
  floor_pose.position.z = -0.5*primitive.dimensions[2];

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(floor_pose);
  collision_object.operation = collision_object.ADD;

  collision_object.header.frame_id = move_group.getPlanningFrame();
  planning_scene.world.collision_objects.push_back(collision_object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);



  


  // subscribing to gazebo feedback messages which contain trajectory information
  ros::Subscriber gazebo_feedback = node_handle.subscribe<control_msgs::FollowJointTrajectoryActionFeedback>(
      "/panda_arm_controller/follow_joint_trajectory/feedback", 1,
      boost::bind(feedbackCallBack, _1, boost::ref(move_group)));

  // cleaning previous file with errors
  std::remove("../../../src/panda_simulation/error.csv");

  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // TASK 1
  int N = 5;
  for (int i = 0; i < N; i++)
  {
    visual_tools.prompt("Press next to continue TASK_1\n");
    visual_tools.deleteAllMarkers();

    ROS_INFO("Step number %i/%i", i + 1, N);

    // current pose of end effector
    Eigen::Isometry3d cur_end_effector = move_group.getCurrentState()->getGlobalLinkTransform("panda_link8");
    geometry_msgs::PoseStamped cur_pose;
    tf::poseEigenToMsg(cur_end_effector, cur_pose.pose);
    visual_tools.publishAxisLabeled(cur_pose.pose, "A", rviz_visual_tools::LARGE);

    // RANDOM POINTS OBTAINED FROM TASK SPACE
    geometry_msgs::PoseStamped trg_pose = move_group.getRandomPose("panda_link8");
    ROS_INFO_STREAM("Next point to plan to : \nt = \n"
                    << trg_pose.pose.position << "\nR = \n"
                    << trg_pose.pose.orientation);
    visual_tools.publishAxisLabeled(trg_pose.pose, "B", rviz_visual_tools::LARGE);
    visual_tools.trigger();

    // setting target pose for move_group
    move_group.setPoseTarget(trg_pose, "panda_link8");

    // do the planning
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Planning state = %s", success ? "SUCCESS" : "FAILED");

    // trajectory visualization
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    // executing planned trajectory
    move_group.execute(my_plan);
  }

  visual_tools.prompt("Press next to finish TASK_1 and return manipulator home\n");
  ROS_INFO("Returning home...");
  visual_tools.deleteAllMarkers();

  // moving manipulator to home position
  geometry_msgs::Pose home_pose;
  home_pose.position.x = 0.5;
  home_pose.position.y = 0;
  home_pose.position.z = 0.5;
  home_pose.orientation.w = 1.0;
  visual_tools.publishAxisLabeled(home_pose, "HOME", rviz_visual_tools::LARGE);

  ROS_INFO_STREAM("Next point to plan to : \nt = \n" << home_pose.position << "\nR = \n" << home_pose.orientation);

  moveit::core::RobotState home_state(*move_group.getCurrentState());
  home_state.setFromIK(joint_model_group, home_pose, 10, 0.2);

  move_group.setJointValueTarget(home_state);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Planning state = %s", success ? "SUCCESS" : "FAILED");

  // trajectory visualization
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // executing planned trajectory
  move_group.execute(my_plan);
  ROS_INFO("Done");

  ros::waitForShutdown();
  return 0;
}
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Duration.h>

#define FOLDED_POSE "folded"
#define RESTING "pre_tap"
#define TAP "tap"


//ros::ServiceClient arm_client;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* arm_client;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "arm_tester");

	ros::NodeHandle nh;

	ROS_DEBUG("Starting AsyncSpinner");
	ros::AsyncSpinner spinner(1);
	spinner.start();


	ROS_INFO_STREAM("Setting up Arm Client");
	arm_client = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("arm_trajectory", true);

//	ROS_INFO_STREAM("Waiting for arm trajectory generator");
//		arm_client->waitForServer();

	ROS_DEBUG_STREAM("setup MoveIT");
	static const std::string PLANNING_GROUP = "arm";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


//	move_group.setGoalPositionTolerance(0.5);
//	move_group.setGoalTolerance(0.5);
//	move_group.setGoalOrientationTolerance(1);
//	ROS_WARN_STREAM("Setting EEF:"<<  move_group.setEndEffector("gripper"));
	ROS_DEBUG_STREAM("MoveIT: Reference frame: "<<move_group.getPlanningFrame());
	ROS_DEBUG_STREAM("MoveIT: End effector link: "<<move_group.getEndEffectorLink());
//	ROS_WARN_STREAM("End Effector:"<< move_group.getEndEffector());
//	ROS_DEBUG("starting main loop");
//	  geometry_msgs::Pose target_pose1;
//	  target_pose1.orientation.w = 1.0;
//	  target_pose1.position.x = -0.132;
//	  target_pose1.position.y = -0.035;
//	  target_pose1.position.z = -0.523;
//	  move_group.setPoseTarget(target_pose1);
//	  move_group.setRandomTarget();
//	move_group.setPositionTarget(0, 0, 0, "bone_3");
	move_group.setNamedTarget(FOLDED_POSE);
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	ROS_WARN_STREAM("Pose:"<<move_group.getCurrentPose().pose.position.x<<move_group.getCurrentPose().pose.position.y<<move_group.getCurrentPose().pose.position.z);
	ROS_WARN_STREAM("oRIENntation: "<<move_group.getCurrentPose().pose.orientation.w<<move_group.getCurrentPose().pose.orientation.x<<move_group.getCurrentPose().pose.orientation.y<<move_group.getCurrentPose().pose.orientation.z);
	//---------my_plan.trajectory_.joint_trajectory (THIS WILL BE SENT).
	bool success = move_group.plan(my_plan);

	move_group.move();
	ROS_WARN_STREAM("Pose:"<<move_group.getCurrentPose().pose.position.x<<move_group.getCurrentPose().pose.position.y<<move_group.getCurrentPose().pose.position.z);

//	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
//	  ROS_WARN_STREAM("Var Pos: "<<current_state->getVariablePosition("servo4_to_bone3")<<" "<<current_state->getVariablePosition(1)<<current_state->getVariablePosition(2)<<" "<<current_state->getVariablePosition(3)<<" "<<current_state->getVariablePosition(4)<<" "<<current_state->getVariablePosition(5));
//	  // Next get the current set of joint values for the group.
//	  std::vector<double> joint_group_positions;
//	  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
//
//	  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
//	  joint_group_positions[3] = -1.0;  // radians
//	  move_group.setJointValueTarget(joint_group_positions);
//
//	  success = move_group.plan(my_plan);
//	  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
//
//	  // Visualize the plan in Rviz
//	  visual_tools.deleteAllMarkers();
//	  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//	  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//	  visual_tools.trigger();
//	visual_tools.prompt("next step");

	return 0;

}


void move_arm(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* arm_client, moveit::planning_interface::MoveGroupInterface& move_group, std::string Position)
{
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	move_group.setNamedTarget(Position);
	bool success = move_group.plan(plan);
	if(success)
	{
		std::vector<control_msgs::JointTolerance> pathTol;
		std::vector<control_msgs::JointTolerance> goalTol;
		ros::Duration tenSec(10.0);
		control_msgs::JointTolerance temp;
		for(int i=0; i<4; ++i)
		{
			temp.position = 0.08726646259;
			temp.velocity =temp.acceleration = 0.17453292519;

			pathTol.push_back(temp);

			temp.position = temp.velocity = 0.06981317007;
			temp.acceleration = 0.03490658503; //2
			goalTol.push_back(temp);
		}
		control_msgs::FollowJointTrajectoryGoal msg;
		msg.trajectory = plan.trajectory_.joint_trajectory;
		msg.path_tolerance = pathTol;
		msg.goal_tolerance = goalTol;
		msg.goal_time_tolerance = ros::Duration(10.0);

		arm_client->sendGoal(msg);
		arm_client->waitForResult(ros::Duration(5.0));
		if(arm_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO_STREAM("Arm successfully moved");
	}
	else
	{
		ROS_INFO_STREAM("MOVEit PLAN FAILURE!");
	}


}

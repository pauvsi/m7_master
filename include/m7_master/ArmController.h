#include <geometry_msgs/Pose.h>
#include "roombaCallback.hpp"

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
//Arm jointstates
#define FOLDED_POSE "folded"
#define RESTING "resting"
#define PRE_TAP "pre_tap"
#define TAP "tap"


geometry_msgs::Pose setArmPose(int roombaIdx)
{
	geometry_msgs::Pose targetPose;
	targetPose.position.x = roombaPose[roombaIdx].pose.pose.position.x;
	targetPose.position.y = roombaPose[roombaIdx].pose.pose.position.y;
	targetPose.position.z = roombaPose[roombaIdx].pose.pose.position.z;
	targetPose.orientation.w = 1.0;

	return targetPose;
}

geometry_msgs::PoseStamped getArmPose(moveit::planning_interface::MoveGroupInterface& move_group)
{
	return move_group.getCurrentPose();
}



void visualizeMoveIt(moveit::planning_interface::MoveGroupInterface::Plan& arm_plan, const robot_state::JointModelGroup* joint_model_group, geometry_msgs::Pose targetPose)
{
//	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("arm_odom_combined");
	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();
	ROS_INFO_STREAM("Visualizing plan as a trajectory line");
	Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();

	visual_tools.publishAxisLabeled(targetPose, "pose1");
	visual_tools.publishText(text_pose, "Pose Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
	visual_tools.publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	visual_tools.prompt("next step");

}

void moveToTarget(moveit::planning_interface::MoveGroupInterface& move_group, const robot_state::JointModelGroup* joint_model_group, geometry_msgs::Pose targetPose, bool visualize)
{
	move_group.setPoseTarget(targetPose);
	moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
	bool success = move_group.plan(arm_plan);

	if(!success)
	{
		ROS_WARN_STREAM("Arm Trajectory Failed!");
		return;
	}
	else
	{
		if(visualize)
		{
			visualizeMoveIt(arm_plan, joint_model_group, targetPose);
		}

		move_group.move();
	}
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

void move_arm( moveit::planning_interface::MoveGroupInterface& move_group, std::string Position)
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
		if(Position.compare(FOLDED_POSE))
			arm_status = 1;
		else if(Position.compare(RESTING))
			arm_status = 2;
		else if(Position.compare(PRE_TAP))
			arm_status = 3;
		else if(Position.compare(TAP))
			arm_status = 4;
//		arm_client->waitForResult(ros::Duration(5.0));
//		if(arm_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO_STREAM("Arm successfully moved");
	}
	else
	{
		ROS_INFO_STREAM("MOVEit PLAN FAILURE!");
	}


}

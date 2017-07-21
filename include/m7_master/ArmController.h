#include <geometry_msgs/Pose.h>
#include "roombaCallback.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


geometry_msgs::Pose setArmPose(int roombaIdx)
{
	geometry_msgs::Pose targetPose;
	targetPose.position.x = roombaPose[roombaIdx].pose.pose.position.x;
	targetPose.position.y = roombaPose[roombaIdx].pose.pose.position.y;
	targetPose.position.z = roombaPose[roombaIdx].pose.pose.position.z;
	targetPose.orientation.w = 1.0;

	return targetPose;
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

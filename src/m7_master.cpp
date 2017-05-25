/*
 * m7_master.cpp
 *
 *  Created on: May 23, 2017
 *      Author: kevin
 */
#include "ros/ros.h"
#include <eigen3/Eigen/Geometry>

#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <pauvsi_trajectory/trajectoryGeneration.h>

#include "../include/m7_master/MasterTypes.h"
#include "../include/m7_master/PID.h"
#include "../include/m7_master/Polynomial.hpp"
#include "../include/m7_master/MasterConfig.h"

#include <pauvsi_trajectory/Physics.h>

#include <deque>

TrajectorySegment requestTrajectory(pauvsi_trajectory::trajectoryGeneration::Request req);

void poseCallback(const geometry_msgs::PoseStampedConstPtr msg);
void twistCallback(const geometry_msgs::TwistStampedConstPtr msg);

DesiredState getDesiredStateAndUpdateGoals(std::deque<HighLevelGoal>& goals);
std_msgs::Float64MultiArray computeMotorForces(DesiredState desired);



std::deque<HighLevelGoal> goalQueue; // stores the high level control goals

PID positionPID, momentPID;
Eigen::Matrix3d P_pos, I_pos, D_pos, P_moment, D_moment;

State state;

ros::ServiceClient traj_client;
ros::Subscriber pose_sub, twist_sub;
ros::Publisher force_pub;

bool executing;

int main(int argc, char **argv){

	ros::init(argc, argv, "mission_7_master_node");

	ros::NodeHandle nh;

	traj_client = nh.serviceClient<pauvsi_trajectory::trajectoryGeneration>("generate_trajectory"); // create a client

	pose_sub = nh.subscribe(POSE_TOPIC, 1, poseCallback);
	twist_sub = nh.subscribe(TWIST_TOPIC, 1, twistCallback);

	force_pub = nh.advertise<std_msgs::Float64MultiArray>(FORCE_TOPIC, 1);

	P_pos << P_POSITION;
	I_pos << I_POSITION;
	D_pos << D_POSITION;
	positionPID = PID(P_pos, I_pos, D_pos);

	P_moment << P_MOMENT;
	D_moment << D_MOMENT;
	momentPID = PID(P_moment, Eigen::MatrixXd::Zero(3, 3), D_moment);

	ros::Rate loop_rate(MASTER_RATE);

	executing = true;
	HighLevelGoal hover;
	hover.hover_pos << -9, -9, 1;
	hover.type = HighLevelGoal::HOVER;
	goalQueue.push_front(hover);

	while(ros::ok())
	{
		ros::spinOnce();

		//update and publish the motor forces
		//WHEN CALLED WITHOUT GOAL WILL HOVER IN PLACE - CAUTION
		if(executing)
		{
			force_pub.publish(computeMotorForces(getDesiredStateAndUpdateGoals(goalQueue)));
		}


		loop_rate.sleep();
	}

	return 0;
}


TrajectorySegment requestTrajectory(pauvsi_trajectory::trajectoryGeneration::Request req){

	pauvsi_trajectory::trajectoryGeneration srv;
	srv.request = req;

	TrajectorySegment seg;

	if (traj_client.call(srv))
	{
		// extract the trajectory
	}
	else
	{
		ROS_ERROR("Failed to call service trajectory generation");
	}

	return seg;
}

std_msgs::Float64MultiArray computeMotorForces(DesiredState desired)
{

}

/*
 * this function checks if goal is obsolete and computes the current desired state
 */
DesiredState getDesiredStateAndUpdateGoals(std::deque<HighLevelGoal>& goals)
{
	DesiredState ds;

	top: // this is for the goto

	// if there is no goal hover in place
	if(goals.size() == 0)
	{
		HighLevelGoal hover;
		hover.type = HighLevelGoal::HOVER;
		hover.hover_pos = state.pos;
		goals.push_front(hover); // a hover goal is indefinite
	}

	//-=-=-=-==-= this section computes the desired state

	if(goals.front().type == HighLevelGoal::HOVER)
	{
		// the desired state is simply a position with no velocity and accel etc
		ds.pos = goals.front().hover_pos;
		ds.vel << 0, 0, 0;
		ds.accel << 0, 0, 0;
		ds.jerk << 0, 0, 0;
		ds.snap << 0, 0, 0;
	}
	else if(goals.front().type == HighLevelGoal::FOLLOW_TRAJECTORY)
	{
		//check if the trajectory is finished computing
		if(ros::Time::now().toSec() > goals.front().traj.finishTime.toSec()){
			goals.pop_front(); // remove the trajectory follow goal. it is finished

			goto top; // re run this function
		}

		double t = ros::Time::now().toSec() - goals.front().traj.startTime.toSec();

		//now we can actually compute the desired state
		ds = polyVal(goals.front().traj.traj, t);
	}

	return ds;
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr msg)
{
	state.pos.x() = msg->pose.position.x;
	state.pos.y() = msg->pose.position.y;
	state.pos.z() = msg->pose.position.z;

	state.attitude.w() = msg->pose.orientation.w;
	state.attitude.x() = msg->pose.orientation.x;
	state.attitude.y() = msg->pose.orientation.y;
	state.attitude.z() = msg->pose.orientation.z;

	state.last_pose_stamp = state.pose_stamp;
	state.pose_stamp = msg->header.stamp;
}
void twistCallback(const geometry_msgs::TwistStampedConstPtr msg)
{
	state.vel.x() = msg->twist.linear.x;
	state.vel.y() = msg->twist.linear.y;
	state.vel.z() = msg->twist.linear.z;

	state.omega.x() = msg->twist.angular.x;
	state.omega.y() = msg->twist.angular.y;
	state.omega.z() = msg->twist.angular.z;

	state.last_twist_stamp = state.twist_stamp;
	state.twist_stamp = msg->header.stamp;
}

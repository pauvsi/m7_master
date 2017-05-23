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

TrajectorySegment requestTrajectory(pauvsi_trajectory::trajectoryGeneration::Request req);

void poseCallback(const geometry_msgs::PoseStampedConstPtr msg);
void twistCallback(const geometry_msgs::TwistStampedConstPtr msg);

std_msgs::Float64MultiArray computeMotorForces();




EfficientTrajectorySegment currentTrajectory, nextTrajectory;
ros::Time currentTrajectoryStartTime;
double currentTrajectoryTransitionTime; // this is the time at which a transition between the two trajectories will take place.

PID positionPID, momentPID;
Eigen::Matrix3d P_pos, I_pos, D_pos, P_moment, D_moment;

State state;

ros::ServiceClient client;
ros::Subscriber pose_sub, twist_sub;
ros::Publisher force_pub;

int main(int argc, char **argv){

	ros::init(argc, argv, "mission_7_master_node");

	ros::NodeHandle nh;

	client = nh.serviceClient<pauvsi_trajectory::trajectoryGeneration>("generate_trajectory"); // create a client

	pose_sub = nh.subscribe(POSE_TOPIC, 1, poseCallback);
	twist_sub = nh.subscribe(TWIST_TOPIC, 1, twistCallback);

	P_pos << P_POSITION;
	I_pos << I_POSITION;
	D_pos << D_POSITION;
	positionPID = PID(P_pos, I_pos, D_pos);

	P_moment << P_MOMENT;
	D_moment << D_MOMENT;
	momentPID = PID(P_moment, Eigen::MatrixXd::Zero(3, 3), D_moment);

	ros::Rate loop_rate(MASTER_RATE);

	while(ros::ok())
	{
		ros::spinOnce();

		//update and publish the motor forces

		loop_rate.sleep();
	}

	return 0;
}


TrajectorySegment requestTrajectory(pauvsi_trajectory::trajectoryGeneration::Request req){

	pauvsi_trajectory::trajectoryGeneration srv;
	srv.request = req;

	TrajectorySegment seg;

	if (client.call(srv))
	{
		// extract the trajectory
	}
	else
	{
		ROS_ERROR("Failed to call service trajectory generation");
	}

	return seg;
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

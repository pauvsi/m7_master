/*
 * m7_master.cpp
 *
 *  Created on: May 23, 2017
 *      Author: kevin
 */
#include "ros/ros.h"
#include <eigen3/Eigen/Geometry>

#include <pauvsi_trajectory/trajectoryGeneration.h>

TrajectorySegment requestTrajectory(pauvsi_trajectory::trajectoryGeneration::Request req);

ros::ServiceClient client;

int main(int argc, char **argv){
	ros::init(argc, argv, "mission_7_master_node");

	ros::NodeHandle nh;

	client = nh.serviceClient<pauvsi_trajectory::trajectoryGeneration>("generate_trajectory"); // create a client
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

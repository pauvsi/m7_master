/*
 * Types.h
 *
 *  Created on: May 11, 2017
 *      Author: kevin
 */

#ifndef MISSION7_MASTER_INCLUDE_MISSION7_MASTER_TYPES_H_
#define MISSION7_MASTER_INCLUDE_MISSION7_MASTER_TYPES_H_

#include <ros/ros.h>

#include <multirotor_trajectory/Types.h>

struct ExecutableTrajectory{
	EfficientTrajectorySegment traj;

	ros::Time startTime;
	ros::Time finishTime;

	ExecutableTrajectory(){

	}

	ExecutableTrajectory(EfficientTrajectorySegment traj, ros::Time start)
	{
		this->startTime = start;
		this->traj = traj;
		this->finishTime = ros::Time(start.toSec() + traj.pos.tf);
	}
};


struct HighLevelGoal{
	enum GoalType{
		FOLLOW_TRAJECTORY,
		HOVER,
		LAND,
	};

	GoalType type;

	//FOLLOW TRAJ
	ExecutableTrajectory traj;

	//HOVER
	Eigen::Vector3d local_hover_pos; //x,y,z coordinates of desired hover
	double hover_yaw; // desired yaw of hover

	//LAND
	Eigen::Vector2d local_land_pos; // the x,y coordinate of the landing position
	double land_yaw; // the desired yaw to land at

};



#endif /* PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TYPES_H_ */

/*
 * Types.h
 *
 *  Created on: May 11, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TYPES_H_
#define PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TYPES_H_

#include <ros/ros.h>



typedef Eigen::VectorXd Polynomial; // highest order coeff first

struct TrajectorySegment{
	Polynomial x, y, z; //polynomial with respect to t
	double tf;
	double t0;

	TrajectorySegment()
	{
		tf = 0;
		t0 = 0;
	}
};

struct EfficientTrajectorySegment{
	TrajectorySegment pos, vel, accel, jerk, snap;
};

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

struct State{
	Eigen::Vector3d pos, vel, omega;

	Eigen::Quaterniond attitude;

	ros::Time pose_stamp, twist_stamp;

	ros::Time last_pose_stamp, last_twist_stamp;

	State()
	{
		pose_stamp = ros::Time::now();
		twist_stamp = pose_stamp;
		last_pose_stamp = pose_stamp;
		last_twist_stamp = pose_stamp;
	}
};

struct HighLevelGoal{
	enum GoalType{
		FOLLOW_TRAJECTORY,
		HOVER,
		TRACK
	};

	GoalType type;

	//FOLLOW TRAJ
	ExecutableTrajectory traj;

	//HOVER
	Eigen::Vector3d pos;

	//TRACK

};




#endif /* PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TYPES_H_ */

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

struct DesiredState{
	Eigen::Vector3d pos, vel, accel, snap, jerk;
};

/*
 * desribes the physical constraints of the quadrotor
 */
struct PhysicalCharacterisics{
	double mass; // kg

	Eigen::Matrix3d J; // diagonal moment of inertia matrix - kg*m^2

	double max_motor_thrust; // N
	double min_motor_thrust; // N

	Eigen::Matrix4d torqueTransition; // matrix which maps motor forces to total force and moments
	Eigen::Matrix4d torqueTransition_inv; // maps total force and moments to motor forces
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
	Eigen::Vector3d pos, vel, accel, omega;

	Eigen::Quaterniond attitude;

	ros::Time pose_stamp, twist_stamp;

	ros::Time last_pose_stamp, last_twist_stamp;

	State(ros::Time t)
	{
		pose_stamp = t;
		twist_stamp = pose_stamp;
		last_pose_stamp = pose_stamp;
		last_twist_stamp = pose_stamp;
	}

	State()
	{

	}
	/*
	 * predict the state forward as best as possible
	 */
	State predictStateForward(ros::Time now)
	{
		State prediction;

		double dt = now.toSec() - pose_stamp.toSec();

		Eigen::Vector3d theta = (omega * dt);

		Eigen::Quaterniond delta = Eigen::AngleAxisd(theta(0), Eigen::Vector3d::UnitX()) *
				Eigen::AngleAxisd(theta(1), Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(theta(2), Eigen::Vector3d::UnitZ());

		prediction.attitude = this->attitude * delta;
		prediction.omega = this->omega;
		prediction.pos = this->pos + this->vel * dt;
		prediction.vel = this->vel;
		prediction.accel = this->accel;

		return prediction;
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
	Eigen::Vector3d hover_pos;

	//TRACK

};




#endif /* PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TYPES_H_ */

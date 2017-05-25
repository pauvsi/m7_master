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

PhysicalCharacterisics phys;

State state;

ros::ServiceClient traj_client;
ros::Subscriber pose_sub, twist_sub;
ros::Publisher force_pub;

bool executing;

int main(int argc, char **argv){

	ros::init(argc, argv, "mission_7_master_node");

	ros::NodeHandle nh;

	state = State(ros::Time::now());

	ROS_DEBUG("starting client");
	traj_client = nh.serviceClient<pauvsi_trajectory::trajectoryGeneration>("generate_trajectory"); // create a client

	ROS_DEBUG("started client");
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

	ROS_DEBUG("setup PIDs");

	phys.J << J_MATRIX;
	phys.mass = MASS;
	phys.max_motor_thrust = MOTOR_FORCE_MAX;
	phys.min_motor_thrust = MOTOR_FORCE_MIN;
	phys.torqueTransition << TORQUE_TRANSITION;
	phys.torqueTransition_inv = phys.torqueTransition.inverse();

	ros::Rate loop_rate(MASTER_RATE);

	HighLevelGoal hover;
	hover.hover_pos << -8, -8, 1;
	hover.type = HighLevelGoal::HOVER;
	goalQueue.push_front(hover);

	ROS_DEBUG("starting main loop");

	while(ros::ok())
	{
		ros::spinOnce();
		ROS_DEBUG("spun");

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
	//ROS_DEBUG("computing motor forces");
	ros::Time t = ros::Time::now();
	State currentState = state.predictStateForward(t);

	Eigen::Vector3d pos_error, vel_error, accel_error;

	pos_error = currentState.pos - desired.pos;
	vel_error = currentState.vel - desired.vel;
	accel_error = currentState.accel - desired.accel;

	// this is to reset the pid controller manually
	// this section runs and resets the pid controller. it differentiates the feedback
	ros::Time time_before_pid_update = positionPID.last_update;
	bool pid_update_bool = positionPID.previous_update;
	Eigen::Vector3d i_sum_before = positionPID.i_sum;

	Eigen::Vector3d accel_fb = positionPID.update(t, pos_error, vel_error);

	Eigen::Vector3d i_sum_after = positionPID.i_sum;

	positionPID.last_update = time_before_pid_update;
	positionPID.i_sum = i_sum_before;
	positionPID.previous_update = pid_update_bool;

	Eigen::Vector3d jerk_fb = positionPID.update(t, vel_error, accel_error);

	positionPID.i_sum = i_sum_after;

	//-==-=-=-=-=--=-=-=- end of feed back differentiation

	Eigen::Vector3d gravity;
	gravity << 0, 0, G;

	Eigen::Vector3d F_inertial = phys.mass * (desired.accel + accel_fb + gravity); // the desired force for the quad to produce with feed back

	double f_total = F_inertial.norm();

	Eigen::Vector3d F_dot_inertial = phys.mass * (desired.jerk + jerk_fb);

	Eigen::Vector3d F_inertial_bar = F_inertial / f_total;

	Eigen::Vector3d F_body_bar;
	F_body_bar << 0, 0, 1.0;

	Eigen::Vector3d F_dot_inertial_bar = (F_dot_inertial / f_total) - (F_inertial * (F_inertial.transpose() * F_dot_inertial)) / (f_total * f_total * f_total);

	Eigen::Vector3d temp = F_inertial_bar.cross(F_dot_inertial_bar);

	Eigen::Vector3d omega_desired;
	omega_desired << temp(0), temp(1), 0;

	Eigen::Quaterniond q_desired;

	double quatNorm = 1 / sqrt(2.0*(1 + F_inertial_bar.transpose() * F_body_bar));

	q_desired.w() = (1 + F_inertial_bar.transpose() * F_body_bar) * quatNorm;


	q_desired.vec() = F_inertial_bar.cross(F_body_bar) * quatNorm;


	//q_desired.normalize();
	//currentState.attitude.normalize();

	//ROS_DEBUG_STREAM("current q: " << currentState.attitude.w() << " "<< currentState.attitude.vec()<< " q_desire: " << q_desired.w() <<" "<< q_desired.vec());

	Eigen::Quaterniond q_error = currentState.attitude.inverse() * q_desired;

	Eigen::Vector3d q_error_vec;
	q_error_vec << q_error.x(), q_error.y(), q_error.z();
	q_error_vec *= q_error.w();

	Eigen::Vector3d omega_error = currentState.omega - omega_desired;

	Eigen::Vector3d moment_desired = momentPID.update(t, q_error_vec, omega_error); // this gets the final moments desired

	Eigen::Vector4d b;
	b << f_total, moment_desired.x(), moment_desired.y(), moment_desired.z();

	Eigen::Vector4d forces = phys.torqueTransition_inv * b;

	ROS_DEBUG_STREAM("f_tot: " << f_total << " moment: " << moment_desired.transpose());

	std_msgs::Float64MultiArray msg;
	//ROS_INFO("here2");
	std::vector<double> vec;
	vec.push_back(forces(0));
	vec.push_back(forces(1));
	vec.push_back(forces(2));
	vec.push_back(forces(3));

	// set up dimensions
	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.layout.dim[0].size = vec.size();
	msg.layout.dim[0].stride = vec.size();
	msg.layout.dim[0].label = "forces"; // or whatever name you typically use to index vec1
	msg.layout.dim[1].size = 1;
	msg.layout.dim[1].stride = 1;
	msg.layout.dim[2].size = 1;
	msg.layout.dim[2].stride = 1;

	//ROS_DEBUG_STREAM("t="<<t);
	msg.data.clear();
	msg.data.insert(msg.data.end(), vec.begin(), vec.end());

	//ROS_DEBUG("computed forces");

	ROS_WARN_STREAM_COND((forces(0) > phys.max_motor_thrust || forces(0) < phys.min_motor_thrust ||
			forces(1) > phys.max_motor_thrust || forces(1) < phys.min_motor_thrust ||
			forces(2) > phys.max_motor_thrust || forces(2) < phys.min_motor_thrust ||
			forces(3) > phys.max_motor_thrust || forces(3) < phys.min_motor_thrust), "FORCE(S) OUT OF BOUNDS " << forces.transpose());

	return msg;
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

	executing = true;

}
void twistCallback(const geometry_msgs::TwistStampedConstPtr msg)
{

	Eigen::Vector3d newVel;
	newVel << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;

	state.accel = (newVel - state.vel) / (msg->header.stamp.toSec() - state.twist_stamp.toSec()); // numerically differentiate to get the accel

	state.vel = newVel;

	state.omega.x() = msg->twist.angular.x;
	state.omega.y() = msg->twist.angular.y;
	state.omega.z() = msg->twist.angular.z;

	state.last_twist_stamp = state.twist_stamp;
	state.twist_stamp = msg->header.stamp;
}

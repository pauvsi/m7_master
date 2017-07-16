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
#include "nav_msgs/Odometry.h"

#include <pauvsi_trajectory/trajectoryGeneration.h>

#include "../include/m7_master/MasterTypes.h"
#include "../include/m7_master/PID.h"
#include "../include/m7_master/Polynomial.hpp"
#include "../include/m7_master/MasterConfig.h"

#include "../include/m7_master/Physics.h"

#include "../include/m7_master/roombaCallback.hpp"


#include <deque>

//set false if estimating state another way
#define USE_GAZEBO_GROUND_TRUTH true


pauvsi_trajectory::trajectoryGeneration::Request generateTrajectoryRequest(WaypointTrajectory traj);
EfficientTrajectorySegment requestTrajectory(pauvsi_trajectory::trajectoryGeneration::Request req);

void poseCallback(const geometry_msgs::PoseStampedConstPtr msg);
void twistCallback(const geometry_msgs::TwistStampedConstPtr msg);
void gtCallback(const nav_msgs::OdometryConstPtr& msg);

DesiredState getDesiredStateAndUpdateGoals(std::deque<HighLevelGoal>& goals);
std_msgs::Float64MultiArray computeMotorForces(DesiredState desired);



std::deque<HighLevelGoal> goalQueue; // stores the high level control goals

PID positionPID, momentPID;
Eigen::Matrix3d P_pos, I_pos, D_pos, P_moment, D_moment;

PhysicalCharacterisics phys;

State state;


ros::ServiceClient traj_client;
#if USE_GAZEBO_GROUND_TRUTH
ros::Subscriber gt_sub;
#else
ros::Subscriber pose_sub, twist_sub;
#endif
ros::Subscriber roomba1, roomba2, roomba3, roomba4, roomba5, roomba6, roomba7, roomba8, roomba9, roomba10;
ros::Subscriber obs1, obs2, obs3, obs4;
ros::Publisher force_pub;



bool executing;

int main(int argc, char **argv){

	ros::init(argc, argv, "mission_7_master_node");

	ros::NodeHandle nh;

	state = State(ros::Time::now());

	ROS_DEBUG("starting client");
	traj_client = nh.serviceClient<pauvsi_trajectory::trajectoryGeneration>("generate_trajectory"); // create a client

	ROS_DEBUG("started client");
#if USE_GAZEBO_GROUND_TRUTH
	gt_sub = nh.subscribe("ground_truth/state", 1, gtCallback);
#else
	pose_sub = nh.subscribe(POSE_TOPIC, 1, poseCallback);
	twist_sub = nh.subscribe(TWIST_TOPIC, 1, twistCallback);
#endif

	roomba1 = nh.subscribe("roomba/roomba1", 1, roombaCallbackOne);
	roomba2 = nh.subscribe("roomba/roomba2", 1, roombaCallbackTwo);
	roomba3 = nh.subscribe("roomba/roomba3", 1, roombaCallbackThree);
	roomba4 = nh.subscribe("roomba/roomba4", 1, roombaCallbackFour);
	roomba5 = nh.subscribe("roomba/roomba5", 1, roombaCallbackFive);
	roomba6 = nh.subscribe("roomba/roomba6", 1, roombaCallbackSix);
	roomba7 = nh.subscribe("roomba/roomba7", 1, roombaCallbackSeven);
	roomba8 = nh.subscribe("roomba/roomba8", 1, roombaCallbackEight);
	roomba9 = nh.subscribe("roomba/roomba9", 1, roombaCallbackNine);
	roomba10 = nh.subscribe("roomba/roomba10", 1, roombaCallbackTen);

	obs1 = nh.subscribe("obstacle/obstacle1", 1, obsCallbackOne);
	obs2 = nh.subscribe("obstacle/obstacle2", 1, obsCallbackTwo);
	obs3 = nh.subscribe("obstacle/obstacle3", 1, obsCallbackThree);
	obs4 = nh.subscribe("obstacle/obstacle4", 1, obsCallbackFour);

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
	hover.hover_pos << -9, -9, .5;
	hover.type = HighLevelGoal::HOVER;
	goalQueue.push_front(hover);


	ROS_DEBUG("generating the request for trajectory");
	WaypointTrajectory traj;
	traj.start.state.pos << -9, -9, 0.5;
	traj.start.state.vel << 0, 0, 0;
	traj.start.state.accel << 0, 0, 0;
	traj.start.state.jerk << 0, 0, 0;
	traj.start.state.snap << 0, 0, 0;

	GeometricConstraint geoConstMax, geoConstMin;
	geoConstMin.type = GeometricConstraint::Z_PLANE_MIN;
	geoConstMin.z_min = 0.0;
	geoConstMax.z_max = 3.0;
	geoConstMax.type = GeometricConstraint::Z_PLANE_MAX;
	traj.start.constraints.push_back(geoConstMin);
	traj.start.constraints.push_back(geoConstMax);

	traj.end.state.pos << 9, 9, 0.5;
	traj.end.state.vel << 0, 0, 0;
	traj.end.state.accel << 0, 0, 0;
	traj.end.state.jerk << 0, 0, 0;
	traj.end.state.snap << 0, 0, 0;

	BasicWaypoint m1;
	m1.pos << 0, 7, 2.5;
	m1.constraints.push_back(geoConstMin);
	m1.constraints.push_back(geoConstMax);
	traj.middle.push_back(m1);

	BasicWaypoint m2;
	m2.pos << 0,-7, 2.5;
	m2.constraints.push_back(geoConstMin);
	m2.constraints.push_back(geoConstMax);
	traj.middle.push_back(m2);

	pauvsi_trajectory::trajectoryGenerationRequest req = generateTrajectoryRequest(traj);

	ROS_DEBUG_STREAM("traj req: " << req);

	HighLevelGoal trajGoal;
	trajGoal.type = HighLevelGoal::FOLLOW_TRAJECTORY;

	ROS_DEBUG("waiting for trajectory generator");
	traj_client.waitForExistence();

	ros::Duration wait(2);
	wait.sleep();

	trajGoal.traj = ExecutableTrajectory(requestTrajectory(req), ros::Time::now() + ros::Duration(5)); // 5 seconds from now

	goalQueue.push_back(trajGoal);

	ros::Duration wait2(2);
	wait2.sleep();

	//traj_client.shutdown();
	//ros::shutdown();
	//return 0;

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


pauvsi_trajectory::trajectoryGeneration::Request generateTrajectoryRequest(WaypointTrajectory traj)
{
	//start
	pauvsi_trajectory::trajectoryGeneration::Request req;
	req.startPosition.x = traj.start.state.pos.x();
	req.startPosition.y = traj.start.state.pos.y();
	req.startPosition.z = traj.start.state.pos.z();

	req.startVelocity.x = traj.start.state.vel.x();
	req.startVelocity.y = traj.start.state.vel.y();
	req.startVelocity.z = traj.start.state.vel.z();

	req.startAcceleration.x = traj.start.state.accel.x();
	req.startAcceleration.y = traj.start.state.accel.y();
	req.startAcceleration.z = traj.start.state.accel.z();

	req.startJerk.x = traj.start.state.jerk.x();
	req.startJerk.y = traj.start.state.jerk.y();
	req.startJerk.z = traj.start.state.jerk.z();

	req.startSnap.x = traj.start.state.snap.x();
	req.startSnap.y = traj.start.state.snap.y();
	req.startSnap.z = traj.start.state.snap.z();

	for(auto e : traj.start.constraints)
	{
		if(e.type == GeometricConstraint::Z_PLANE_MAX)
		{
			req.startMaxZ = e.z_max;
		}
		else if(e.type == GeometricConstraint::Z_PLANE_MIN)
		{
			req.startMinZ = e.z_min;
		}
	}

	//end
	req.goalPosition.x = traj.end.state.pos.x();
	req.goalPosition.y = traj.end.state.pos.y();
	req.goalPosition.z = traj.end.state.pos.z();

	req.goalVelocity.x = traj.end.state.vel.x();
	req.goalVelocity.y = traj.end.state.vel.y();
	req.goalVelocity.z = traj.end.state.vel.z();

	req.goalAcceleration.x = traj.end.state.accel.x();
	req.goalAcceleration.y = traj.end.state.accel.y();
	req.goalAcceleration.z = traj.end.state.accel.z();

	req.goalJerk.x = traj.end.state.jerk.x();
	req.goalJerk.y = traj.end.state.jerk.y();
	req.goalJerk.z = traj.end.state.jerk.z();

	req.goalSnap.x = traj.end.state.snap.x();
	req.goalSnap.y = traj.end.state.snap.y();
	req.goalSnap.z = traj.end.state.snap.z();

	// the end has no constraints that do anything

	//middle

	//req.middle_size = traj.middle.size();

	for(auto e : traj.middle)
	{
		geometry_msgs::Vector3 vec;
		vec.x = e.pos.x();
		vec.y = e.pos.y();
		vec.z = e.pos.z();
		req.middle.push_back(vec);
		req.middleGeometricConstraints.push_back(e.constraints.at(0).z_min);
		req.middleGeometricConstraints.push_back(e.constraints.at(1).z_max);
	}

	return req;
}

EfficientTrajectorySegment requestTrajectory(pauvsi_trajectory::trajectoryGeneration::Request req){

	pauvsi_trajectory::trajectoryGeneration srv;
	srv.request = req;

	EfficientTrajectorySegment eseg;

	if (traj_client.call(srv))
	{
		// extract the trajectory
		ROS_DEBUG_STREAM("traj response: " << srv.response);
		TrajectorySegment seg;
		seg.t0 = 0;
		seg.tf = srv.response.tf;

		Polynomial poly = Polynomial(srv.response.trajectory.layout.dim[1].size);

		for(int i = 0; i < srv.response.trajectory.layout.dim[1].size; i++)
		{
			poly(i) = srv.response.trajectory.data.at(srv.response.trajectory.layout.dim[1].size * 0 + i);
		}
		seg.x = poly;
		for(int i = 0; i < srv.response.trajectory.layout.dim[1].size; i++)
		{
			poly(i) = srv.response.trajectory.data.at(srv.response.trajectory.layout.dim[1].size * 1 + i);
		}
		seg.y = poly;
		for(int i = 0; i < srv.response.trajectory.layout.dim[1].size; i++)
		{
			poly(i) = srv.response.trajectory.data.at(srv.response.trajectory.layout.dim[1].size * 2 + i);
		}
		seg.z = poly;

		eseg = EfficientTrajectorySegment(seg);
	}
	else
	{
		ROS_ERROR("Failed to call service trajectory generation");
	}

	return eseg;
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

	ROS_WARN_STREAM_COND((forces(0) > MOTOR_ABS_MAX || forces(0) < MOTOR_ABS_MIN ||
			forces(1) > MOTOR_ABS_MAX || forces(1) < MOTOR_ABS_MIN ||
			forces(2) > MOTOR_ABS_MAX || forces(2) < MOTOR_ABS_MIN ||
			forces(3) > MOTOR_ABS_MAX || forces(3) < MOTOR_ABS_MIN), "FORCE(S) OUT OF BOUNDS " << forces.transpose());

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

		// check to see if a new goal is ready to be processed
		if(goals.size() > 1)
		{
			// if there is a trajectory to follow
			if((goals.at(1).type == HighLevelGoal::FOLLOW_TRAJECTORY) && (goals.at(1).traj.startTime.toSec() <= ros::Time::now().toSec()))
			{
				ROS_DEBUG("exectuting trajectory");
				goals.pop_front();
			}
		}

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

		ROS_DEBUG_STREAM("desired acceleration: " << ds.accel.transpose());
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

void gtCallback(const nav_msgs::OdometryConstPtr& msg)
{
	Eigen::Vector3d newVel;
	newVel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;

	state.accel = (newVel - state.vel) / (msg->header.stamp.toSec() - state.twist_stamp.toSec()); // numerically differentiate to get the accel

	state.vel = newVel;

	state.omega.x() = msg->twist.twist.angular.x;
	state.omega.y() = msg->twist.twist.angular.y;
	state.omega.z() = msg->twist.twist.angular.z;

	state.last_twist_stamp = state.twist_stamp;
	state.twist_stamp = msg->header.stamp;

	state.pos.x() = msg->pose.pose.position.x;
	state.pos.y() = msg->pose.pose.position.y;
	state.pos.z() = msg->pose.pose.position.z;

	state.attitude.w() = msg->pose.pose.orientation.w;
	state.attitude.x() = msg->pose.pose.orientation.x;
	state.attitude.y() = msg->pose.pose.orientation.y;
	state.attitude.z() = msg->pose.pose.orientation.z;

	state.last_pose_stamp = state.pose_stamp;
	state.pose_stamp = msg->header.stamp;

	executing = true;
}





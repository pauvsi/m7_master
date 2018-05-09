/*
 * M7Master.cpp
 *
 *  Created on: May 8, 2018
 *      Author: kevin
 */

#include "M7Master.h"

M7Master::M7Master() {
	ros::NodeHandle nh;

	//set up subs and pubs
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &M7Master::fcu_state_cb, this);
	local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");


	// wait for a connection to the flight controller
	//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(20.0);

	ROS_INFO("waiting for connection to Flight Controller.");
	// wait for FCU connection
	while(ros::ok() && !fcu_state.connected){
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("Successfully connected to FCU.");

	//TODO wait for good state estimation

	//TODO wait for start signal from user


	//set up timer events
	arm_offboard_timer = nh.createTimer(ros::Rate(0.2), &M7Master::arming_timer_callback, this);

	this->main_loop();
}

M7Master::~M7Master() {

}

void M7Master::main_loop(){

	while(ros::ok()){
		ros::spinOnce(); // get all messages and call timers
	}

	//program has been killed

}


void M7Master::arming_timer_callback(const ros::TimerEvent& msg){
	ROS_INFO("arming and setting to offboard mode.");

	this->arm_flight_controller();
	this->set_to_offboard_mode();
}

void M7Master::arm_flight_controller(){
	if(!fcu_state.armed){
		mavros_msgs::CommandBool arm_cmd;
		arm_cmd.request.value = true;

		if( arming_client.call(arm_cmd) && arm_cmd.response.success){
			ROS_INFO("Vehicle armed");
		}
		else{

		}
	}
}

void M7Master::set_to_offboard_mode(){
	if( fcu_state.mode != "OFFBOARD"){

		mavros_msgs::SetMode offb_set_mode;
		offb_set_mode.request.custom_mode = "OFFBOARD";

		if( set_mode_client.call(offb_set_mode) &&
				offb_set_mode.response.mode_sent){
			ROS_INFO("Offboard enabled");
		}else{
			ROS_ERROR("failed to set to offboard mode.");
		}
	}
}

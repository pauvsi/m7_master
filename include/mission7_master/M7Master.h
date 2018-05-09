/*
 * M7Master.h
 *
 *  Created on: May 8, 2018
 *      Author: kevin
 */

#ifndef MISSION7_MASTER_INCLUDE_MISSION7_MASTER_M7MASTER_H_
#define MISSION7_MASTER_INCLUDE_MISSION7_MASTER_M7MASTER_H_

#include "multirotor_trajectory/Polynomial.hpp"

#include <mavros/mavros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>

class M7Master {
public:

	mavros_msgs::State fcu_state; // determine connection etc.

	ros::Publisher local_pos_pub;
	ros::ServiceClient arming_client;
	ros::ServiceClient set_mode_client;

	ros::Timer arm_offboard_timer;

	M7Master();
	virtual ~M7Master();

	void fcu_state_cb(const mavros_msgs::State::ConstPtr& msg){this->fcu_state = *msg;}

	void main_loop();

	void arm_flight_controller();

	void arming_timer_callback(const ros::TimerEvent& msg);
};

#endif /* MISSION7_MASTER_INCLUDE_MISSION7_MASTER_M7MASTER_H_ */

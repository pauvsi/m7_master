/*
 * master_node.cpp
 *
 *  Created on: May 8, 2018
 *      Author: kevin
 */
#include "ros/ros.h"
#include "../include/mission7_master/M7Master.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "mission_7_master_node");

	ros::NodeHandle nh;

	M7Master master;

}


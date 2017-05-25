/*
 * Polynomial.h
 *
 *  Created on: Apr 14, 2017
 *      Author: pauvsi
 */

#ifndef PAUVSI_M7_PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_POLYNOMIAL_HPP_
#define PAUVSI_M7_PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_POLYNOMIAL_HPP_

#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iostream>
#include "ros/ros.h"

#include "MasterTypes.h"

/*
 * finds the derivative of polynomial coefficients
 */
Polynomial polyDer(Polynomial in)
{
	int size = in.size();
	Polynomial new_poly(size - 1, 1);
	for(int i=0; i < size-1; i++){
		new_poly(i) = ((size - 1 - i)*in(i));
	}
	return new_poly;
}

TrajectorySegment polyDer(TrajectorySegment ts)
{
	ts.x = polyDer(ts.x);
	ts.y = polyDer(ts.y);
	ts.z = polyDer(ts.z);
	return ts;
}

double polyVal(Polynomial in, double t)
{
	double result=0;
	double t_agg = 1;
	for (int i = in.size() - 1; i >= 0; i--){
		result += t_agg * in(i);
		t_agg *= t;
	}
	return result;
}

Eigen::Vector3d polyVal(TrajectorySegment ts, double t)
{
	Eigen::Vector3d vec;

	vec << polyVal(ts.x, t), polyVal(ts.y, t), polyVal(ts.z, t);

	return vec;
}

DesiredState polyVal(EfficientTrajectorySegment ets, double t)
{
	DesiredState ds;
	ds.pos = polyVal(ets.pos, t);
	ds.vel = polyVal(ets.vel, t);
	ds.accel = polyVal(ets.accel, t);
	ds.jerk = polyVal(ets.jerk, t);
	ds.snap = polyVal(ets.snap, t);

	return ds;
}



#endif /* PAUVSI_M7_PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_POLYNOMIAL_HPP_ */

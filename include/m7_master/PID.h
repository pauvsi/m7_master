/*
 * PID.h
 *
 *  Created on: May 23, 2017
 *      Author: kevin
 */

#ifndef M7_MASTER_INCLUDE_M7_MASTER_PID_H_
#define M7_MASTER_INCLUDE_M7_MASTER_PID_H_

#include <eigen3/Eigen/Eigen>

class PID{
public:

	Eigen::Matrix3d P, I, D;
	bool noP, noI, noD;

	Eigen::Vector3d i_sum;

	ros::Time last_update;
	bool previous_update;

	PID()
	{
		noP = true;
		noI = true;
		noD = true;
		previous_update = false;
	}

	PID(Eigen::Matrix3d P, Eigen::Matrix3d I, Eigen::Matrix3d D){
		if(P.squaredNorm() == 0)
		{
			noP = true;
		}
		else
		{
			noP = false;
			this->P = P;
		}
		if(I.squaredNorm() == 0)
		{
			noI = true;
		}
		else
		{
			noI = false;
			this->I = I;
		}
		if(D.squaredNorm() == 0)
		{
			noD = true;
		}
		else
		{
			noD = false;
			this->D = D;
		}
		previous_update = false;
	}

	Eigen::Vector3d update(ros::Time now, Eigen::Vector3d error, Eigen::Vector3d error_dot)
	{
		Eigen::Vector3d fb;

		double dt = 0;
		if(previous_update){dt = now.toSec() - last_update.toSec();}
		last_update = now;

		if(!noP)
		{
			fb += -P * error;
		}
		if(!noI)
		{
			fb += -I * (i_sum + (dt)*error);
		}
		if(!noD)
		{
			fb += -D * error_dot;
		}

		previous_update = true;

		return fb;
	}

	void reset(){
		previous_update = false;
		i_sum = Eigen::VectorXd::Zero(3);
	}
};


#endif /* M7_MASTER_INCLUDE_M7_MASTER_PID_H_ */

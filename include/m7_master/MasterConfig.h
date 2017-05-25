/*
 * MasterConfig.h
 *
 *  Created on: May 23, 2017
 *      Author: kevin
 */

#ifndef M7_MASTER_INCLUDE_M7_MASTER_MASTERCONFIG_H_
#define M7_MASTER_INCLUDE_M7_MASTER_MASTERCONFIG_H_


#define POSE_TOPIC "state/pose"
#define TWIST_TOPIC "state/twist"

#define FORCE_TOPIC "forceRequest"

#define MASTER_RATE 100

#define P_POSITION 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1
#define I_POSITION 0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001
#define D_POSITION 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1

#define P_MOMENT 0.5, 0, 0, 0, 1, 0, 0, 0, 0.5
#define D_MOMENT 0.75, 0, 0, 0, 0.75, 0, 0, 0, 0.75

#endif /* M7_MASTER_INCLUDE_M7_MASTER_MASTERCONFIG_H_ */

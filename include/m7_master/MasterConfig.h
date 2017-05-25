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

#define P_POSITION 1,     0, 0, 0,      1,     0, 0, 0,    1
#define I_POSITION 0,    0, 0, 0,       0      , 0, 0, 0,      0
#define D_POSITION 1.2   , 0, 0, 0,     1.2    , 0, 0, 0,     1.5

#define P_MOMENT 2,      0, 0, 0,      2      , 0, 0, 0,      -2
#define D_MOMENT 4,      0, 0, 0,      4,      0, 0, 0,      4

#endif /* M7_MASTER_INCLUDE_M7_MASTER_MASTERCONFIG_H_ */

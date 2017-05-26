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

#define P_POSITION 15,     0, 0, 0,      15,     0, 0, 0,    15
#define I_POSITION 4,    0, 0, 0,       4      , 0, 0, 0,      4
#define D_POSITION 4   , 0, 0, 0,    4    , 0, 0, 0,     4

#define P_MOMENT 5,      0, 0, 0,      5      , 0, 0, 0,      -5
#define D_MOMENT 15,      0, 0, 0,      15,      0, 0, 0,      15

#endif /* M7_MASTER_INCLUDE_M7_MASTER_MASTERCONFIG_H_ */

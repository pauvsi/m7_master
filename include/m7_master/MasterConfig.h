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

#define P_POSITION 7,     0, 0, 0,      7,     0, 0, 0,    7
#define I_POSITION 4,    0, 0, 0,       4      , 0, 0, 0,      4
#define D_POSITION 2   , 0, 0, 0,    2    , 0, 0, 0,     2

#define P_MOMENT 3,      0, 0, 0,      3      , 0, 0, 0,      -3
#define D_MOMENT 9,      0, 0, 0,      9,      0, 0, 0,      9

#endif /* M7_MASTER_INCLUDE_M7_MASTER_MASTERCONFIG_H_ */

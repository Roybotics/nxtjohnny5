/**
 *******************************************************************************
 **	FILE NAME : nxt_config.h
 **
 **	ABSTRUCT  : NXT device configuration
 *******************************************************************************
 **/

#ifndef _NXT_CONFIG_H_
#define _NXT_CONFIG_H_

#include "ecrobot_interface.h"

/* NXT motor port configuration */
#define PORT_MOTOR_R	NXT_PORT_A
#define PORT_MOTOR_L	NXT_PORT_C
#define PORT_MOTOR_H	NXT_PORT_B
/* NXT sensor port configuration */
#define PORT_SONAR		NXT_PORT_S1
#define PORT_CAM		NXT_PORT_S2
#define PORT_ACCL		NXT_PORT_S3
#define PORT_GYRO		NXT_PORT_S4
/* NXT Bluetooth configuration */
#define BT_PASS_KEY		"1234"

#endif

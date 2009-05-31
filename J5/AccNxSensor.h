/**
 *******************************************************************************
 **	FILE NAME : AccNxSensor.h
 **
 **	ABSTRACT  : Mindsensors accelerometer driver.
 ** Acknowledgments: based on code from Takashi Chikamasa.
 *******************************************************************************
 **/

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* Prototypes */
void init_mindsensor_accel_sensor(U8 port_id);
void get_mindsensor_accel_sensor(U8 port_id, S16 *buf);
S32  getaccelnorm(S16 *buf);
void term_mindsensor_accel_sensor(U8 port_id);

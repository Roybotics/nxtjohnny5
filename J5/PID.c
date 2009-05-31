/**
 ******************************************************************************
 **	FILE NAME : PID.c
 **
 **	ABSTRACT  : PID control.
 **
 *******************************************************************************
 **/

#include "mymath.h"
#include "PID.h"

/*
 *
 */
inline void PIDinit(PIDctrl* pid, float kp, float ki, float kd, float kid)
{
	pid->lerror = 0;
	pid->ierror = 0;
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->kid = kid;
}

/*
 * Ziegler–Nichols tuning method.
 * As in the method above, the Ki and Kd gains are first set to zero.
 * The P gain is increased until it reaches the critical gain, Kc,
 * at which the output of the loop starts to oscillate.
 * Kc and the oscillation period Pc are used to set the gains.
 * (http://en.wikipedia.org/wiki/PID_controller)
 */
inline void PIDinitZN(PIDctrl* pid, float kc, float pc, float kid)
{
	pid->lerror = 0;
	pid->ierror = 0;
	pid->kp = 0.6 * kc;
	if (pc != 0)
	{
		pid->ki = 2 * pid->kp / pc;
		pid->kd = pid->kp * pc / 8;
	}
	pid->kid = kid;
}

inline float PIDstep(PIDctrl* pid, float error)
{
	pid->error = error;

	float output = 0;
	output += pid->kp * error;

	pid->ierror = pid->kid * pid->ierror + error;
	output += pid->ki * pid->ierror;

	output += pid->kd * (error - pid->lerror);

	pid->lerror = pid->error;

	return output;
}

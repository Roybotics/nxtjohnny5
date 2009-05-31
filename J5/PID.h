/**
 ******************************************************************************
 **	FILE NAME : PID.h
 **
 **	ABSTRACT  : PID control.
 **
 *******************************************************************************
 **/

/*
 * Control state stored in special struct:
 */
typedef struct
{
	//state:
	float error;
	float lerror;
	float ierror;

	//parameters:
	float kp;
	float ki;
	float kd;
	float kid;

} PIDctrl;

void PIDinitZN(PIDctrl* pid, float kc, float pc, float kid);
void PIDinit(PIDctrl* pid, float kp, float pi, float kd, float kid);
float PIDstep(PIDctrl* pid, float error);


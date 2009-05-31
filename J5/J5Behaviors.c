/**
 ******************************************************************************
 **	FILE NAME : J5Behaviors.c
 **
 **	ABSTRACT  : J5 (Johnny 5) Behaviors
 **
 *******************************************************************************
 **/
#include "mymath.h"
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "nxt_config.h"
#include "J5Behaviors.h"
#include "NxtCam.h"
#include "PID.h"

/*get_mindsensor_accel_sensor(PORT_ACCL,state.accl);
 F32 acclnorm = state.accl[3];
 state.acclnorm = acclnorm;
 state.acclavg = 0.99*state.acclavg + 0.01*acclnorm;
 state.accldelta = acclnorm - state.acclavg;
 ecrobot_show_int(acclnorm);/**/

void datalogger(float dataarray[])
{
	U8 buffer[32];

	for (int i = 0; i < 8; i++)
	{
		*((F32*) (&buffer[i * 4])) = (F32) dataarray[i];
	}

	ecrobot_send_bt_packet(buffer, 32);
}

inline void head_control(State* state)
{
	if (state->seektargetheadangle == 1)
	{
		if (state->targetheadangle < -40)
			state->targetheadangle = -40;
		else if (state->targetheadangle > 90)
			state->targetheadangle = 90;
		int error = state->targetheadangle - state->headangle;

		if (ABS(error)>3)
			state->pwmH = error;
		else
		{
			state->pwmH = 0;
			state->seektargetheadangle = 0;
		}
	}

	if (state->headangle < -30) // head up
		state->pwmH = CLAMP(state->pwmH,0,100);
	else if (state->headangle > 110) // head down
		state->pwmH = CLAMP(state->pwmH,-100,0);

	nxt_motor_set_speed(PORT_MOTOR_H, state->pwmH, 1);
}

inline void target2effective(State* state)
{
	if (state->update == 1)
	{
		state->forward = state->targetforward;
		state->turn = state->targetturn;
		state->update = 0;
	}
}

inline void sonarslowdown(State* state)
{
	if (state->sonar <= 25 && state->forward > 0)
	{
		// Slow down J5:
		state->forward = 0.9 * state->forward;
	}
}

int collision = 0;
inline void acclslowdown(State* state)
{
	if (collision > 0)
		collision--;

	int sign = SIGN(state->forward);
	float indicator = (F32) (state->pwmLR * sign);
	float absforward = (F32) (state->forward * sign);

	float log[8] =
	{ (float) (indicator), (float) (state->pwmLR), (float) (state->forward),
			(float) (absforward), (float) (10 * sign), (float) (0),
			(float) (0), (float) (0) };
	datalogger(log);

	if (sign == 0)
	{
		if (collision == 0 && state->targetforward == 0 && ABS(state->pwmLR) > 50)
		{
			state->targetforward = -SIGN(state->pwmLR) * 30;
			state->targetturn = 0;
			state->update = 1;

			collision = 100;
		}
	}
	else // sign = +/-1
	{
		if (collision == 0 && (indicator > 50))
		{
			state->targetforward = -sign * 30;
			state->targetturn = -state->targetturn;
			state->update = 1;

			collision = 100;
		}
	}

	if (collision > 0)
	{
		state->forward = 0.99 * state->forward;
		state->turn = 0.90 * state->turn;
	}

	if (collision == 1)
	{
		state->targetforward = 0;
		state->targetturn = 0;
		state->update = 1;
	}

}

inline void standstillbehavior(State* state)
{
	target2effective(state);
	//sonarslowdown(state);
	acclslowdown(state);
	trackredball(state);
}

static F32 stablestandstillcounter = 0;
inline void stablestandstillbehavior(State* state)
{

	stablestandstillcounter += 0.01;
	state->targetturn = (U8)((int) (100 * cos(stablestandstillcounter)));
	state->update = 1;
	standstillbehavior(state);
}


static int pidRedVrtInit = 0;
static PIDctrl pidRedVrt;
static PIDctrl pidRedHrz;

inline void trackredball(State* state)
{
	if(pidRedVrtInit==0)
	{
		PIDinitZN(&pidRedVrt,0.50,10,0.99);
		PIDinit(&pidRedHrz,2.5 ,0,0,0.99);
		pidRedVrtInit=1;
	}

	int rectindex = getbiggestrect(0, -1);

	if (rectindex >= 0)
	{
		int x = getX(rectindex);
		int y = getY(rectindex);

		int vrterror = y - 72;

		float vrtoutput = PIDstep(&pidRedVrt, vrterror);
		state->pwmH = (S8)CLAMP(vrtoutput, -100, 100);

		int hrzerror = -(x - 176 / 2);
		float hrzoutput = PIDstep(&pidRedHrz, hrzerror);
		state->targetturn = (S8)CLAMP(hrzoutput,-100,100);

		float beta = (90-state->headangle) + (x-72)*(23.0/72.0);
		float factor = (2*PI)/360;

		float d = 21*tan(beta*factor);

		state->targetforward = (S8)CLAMP(2*(d-10),-60,60);

		state->update = 1;

		ecrobot_debug2((state->headangle), (beta), d);
		//ecrobot_debug2((y), (vrterror), state->pwmH);
	}
	/**/

}

inline void ballpositionmodel(int angle, int x, int y)
{

}

/******************************** END OF FILE ********************************/

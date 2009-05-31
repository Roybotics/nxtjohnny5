/**
 ******************************************************************************
 **	FILE NAME : J5.c
 **
 **	ABSTRACT  : J5 (Johnny 5) based on NXTway-GS (with a HiTechnic Gyro Sensor)
 **	            two-wheeled self-balancing autonomous robot with camera
 **	            receiving commands via Bluetooth.
 ** Acknowledgments: based on code from Takashi Chikamasa.
 *******************************************************************************
 **/
#include "mymath.h"
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "balancer.h"    // NXTway-GS C API header file
#include "nxt_config.h"  // Nxt port configuration
#include "J5Behaviors.h" // J5 Behaviors
#include "AccNxSensor.h" // Accelerometer driver
#include "NxtCam.h"      // Camera driver

/*============================================================================
 * MACRO DEFINITIONS
 *===========================================================================*/
typedef enum
{
	INIT_MODE, /* system initialize mode */
	GYROCAL_MODE, /* gyro sensor offset calibration mode */
	HEADCAL_MODE, /* head calibration mode */
	CONTROL_MODE
/* balance and RC control mode */
} MODE_ENUM;

#define  BT_RCV_BUF_SIZE (32) /* 32bytes fixed for NXT GamePad */

/*============================================================================
 * DATA DEFINITIONS
 *===========================================================================*/
volatile static MODE_ENUM J5mode = INIT_MODE; /* J5 mode */
static void (*J5BehaviorArray[])(State*) =
{ standstillbehavior, stablestandstillbehavior };
static void (*J5BehaviorFunction)(State*) = &standstillbehavior;
volatile static State state;

/*============================================================================
 * FUNCTIONS
 *===========================================================================*/
/*============================================================================
 * Embedded Coder Robot hook functions
 *===========================================================================*/
//*****************************************************************************
// FUNCTION		: ecrobot_device_initialize
// ARGUMENT		: none
// RETURN		: none
// DESCRIPTION 	: ECRobot device init hook function
//*****************************************************************************
void ecrobot_device_initialize(void)
{
	ecrobot_init_sonar_sensor(PORT_SONAR);
	ecrobot_init_bt_slave(BT_PASS_KEY);
	//init_mindsensor_accel_sensor(PORT_ACCL);

}

//*****************************************************************************
// FUNCTION		: ecrobot_device_terminate
// ARGUMENT		: none
// RETURN		: none
// DESCRIPTION 	: ECRobot device term hook function
//*****************************************************************************
void ecrobot_device_terminate(void)
{
	nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
	nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
	ecrobot_term_sonar_sensor(PORT_SONAR);
	ecrobot_term_bt_connection();
	//term_mindsensor_accel_sensor(PORT_ACCL);
	term_nxtcam(PORT_CAM);
}
/*============================================================================
 * TOPPERS OSEK specific Function/Tasks
 *===========================================================================*/
DeclareCounter(SysTimerCnt);

//*****************************************************************************
// FUNCTION		: user_1ms_isr_type2
// ARGUMENT		: none
// RETURN		: none
// DESCRIPTION 	: 1msec periodical OSEK type 2 ISR
//*****************************************************************************
void user_1ms_isr_type2(void)
{
	/* Increment System Timer Count to activate periodical Tasks */
	(void) SignalCounter(SysTimerCnt);
}

void dataloggerS32(S32 dataarray[], int length)
{
	U8 buffer[4 * length];

	for (int i = 0; i < length; i++)
	{
		*((S32*) (&buffer[i * 4])) = (S32) dataarray[i];
	}

	ecrobot_send_bt_packet(buffer, 4 * length);
}

void dataloggerF32(F32 dataarray[], int length)
{
	U8 buffer[4 * length];

	for (int i = 0; i < length; i++)
	{
		*((F32*) (&buffer[i * 4])) = (F32) dataarray[i];
	}

	ecrobot_send_bt_packet(buffer, 4 * length);
}

void datalogger(F32 dataarray[])
{
	U8 buffer[32];

	for (int i = 0; i < 8; i++)
	{
		*((F32*) (&buffer[i * 4])) = (F32) dataarray[i];
	}

	ecrobot_send_bt_packet(buffer, 32);
}

void dataloggerS8(S8 dataarray[], int length)
{
	S8 buffer[length];

	for (int i = 0; i < length; i++)
	{
		*((S8*) (&buffer[i])) = (S8) dataarray[i];
	}

	ecrobot_send_bt_packet(buffer, length);
}

//*****************************************************************************
// TASK			: OSEK_Task_ts1
// ARGUMENT		: none
// RETURN		: none
// DESCRIPTION 	: 4msec periodical Task and controls NXTway-GS
//                INIT_MODE
//
//                CAL_MODE
//
//                CONTROL_MODE
//*****************************************************************************
TASK(OSEK_Task_ts1)
{
	static U32 gyro_offset; /* gyro sensor offset value */
	static U32 avg_cnt; /* average count to calc gyro offset */
	static U32 cal_start_time; /* calibration start time */
	static int last_head_motorcount;

	S8 pwmL, pwmR;

	switch (J5mode)
	{
	case (INIT_MODE):

		init_nxtcam(PORT_CAM);
		//send_nxtcam_command(PORT_CAM,SORT_BY_SIZE);
		//send_nxtcam_command(PORT_CAM,TRACK_OBJECTS);
		SINT ret = send_nxtcam_command(PORT_CAM,ENABLE_TRACKING);
		if (ret == 0)
		{
			/**/

			gyro_offset = 0;
			avg_cnt = 0;

			balance_init(); /* NXTway-GS C API initialize function */
			nxt_motor_set_count(PORT_MOTOR_L, 0); /* reset left motor count */
			nxt_motor_set_count(PORT_MOTOR_R, 0); /* reset right motor count */
			cal_start_time = ecrobot_get_systick_ms();
			J5mode = GYROCAL_MODE;
		}
		break;

	case (GYROCAL_MODE):
		gyro_offset += (U32) ecrobot_get_gyro_sensor(PORT_GYRO);
		avg_cnt++;
		if ((ecrobot_get_systick_ms() - cal_start_time) >= 1000U)
		{
			/* 1000msec later, start balancing */
			gyro_offset /= avg_cnt;
			ecrobot_sound_tone(440U, 500U, 30U); /* beep a tone */
			J5mode = HEADCAL_MODE;
			cal_start_time = ecrobot_get_systick_ms();
			state.gyro_offset = gyro_offset;
		}
		break;

	case (HEADCAL_MODE):

		nxt_motor_set_speed(PORT_MOTOR_H, -40, 1);

		state.headangle = nxt_motor_get_count(PORT_MOTOR_H);

		//if ((ecrobot_get_systick_ms() - cal_start_time) >= 1000U)
		if (((ABS(last_head_motorcount - state.headangle) == 0)
				&& ((ecrobot_get_systick_ms() - cal_start_time) >= 100U))
				|| ((ecrobot_get_systick_ms() - cal_start_time) >= 30000U))
		{
			nxt_motor_set_speed(PORT_MOTOR_H, 0, 1);
			nxt_motor_set_count(PORT_MOTOR_H, -45);
			state.targetheadangle = 60;
			state.seektargetheadangle = 1;
			ecrobot_sound_tone(880U, 500U, 30U); /* beep a tone */
			J5mode = CONTROL_MODE;
			state.gyro_offset = gyro_offset;
		}

		last_head_motorcount = state.headangle;
		ecrobot_show_int(last_head_motorcount);
		break;

	case (CONTROL_MODE):

		state.gyro = ecrobot_get_gyro_sensor(PORT_GYRO);

		state.motorcountL = nxt_motor_get_count(PORT_MOTOR_L);
		state.motorcountR = nxt_motor_get_count(PORT_MOTOR_R);

		/* NXTway-GS C API balance control function (has to be invoked in 4msec period) */
		balance_control((F32) (-state.forward), (F32) (state.turn),
				(F32) (state.gyro), (F32) (state.gyro_offset),
				(F32) (state.motorcountL), (F32) (state.motorcountR),
				(F32) ecrobot_get_battery_voltage(), &pwmL, &pwmR);

		nxt_motor_set_speed(PORT_MOTOR_L, pwmL, 1);
		nxt_motor_set_speed(PORT_MOTOR_R, pwmR, 1);

		state.pwmL = -pwmL;
		state.pwmR = -pwmR;
		state.pwmLR = (pwmL + pwmR) / 2;

		//ecrobot_bt_data_logger(state.forward, state.turn); /* Bluetooth data logger */
		break;

	default:
		/* unexpected mode */
		nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
		nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
		break;
	}

	TerminateTask(); /* terminates this task and executed by System Timer Count */
}

//*****************************************************************************
// TASK			: OSEK_Task_ts2
// ARGUMENT		: none
// RETURN		: none
// DESCRIPTION 	: 30msec periodical Task for main decision making
//*****************************************************************************
int counter = 0;
TASK(OSEK_Task_ts2)
{
	int Tcounter;
	if ((J5mode == CONTROL_MODE))
	{
		counter++;
		state.sonar = ecrobot_get_sonar_sensor(PORT_SONAR);
		Tcounter = request(PORT_CAM);
		state.camdata = getdata();
		state.headangle = nxt_motor_get_count(PORT_MOTOR_H);

		//ecrobot_show_int((F32)(state.camdata[0]));
		//ecrobot_show_int((F32)Tcounter);
		//ecrobot_show_int((F32)state.camdata[1]);
		//ecrobot_debug1((Tcounter), (state.camdata[0]), getbiggestrect(0, -1));
		//ecrobot_show_int(state.motorcountH);



		J5BehaviorFunction(&state);

		head_control(&state);
	}/**/

	TerminateTask(); /* terminates this task and executed by System Timer Count */
}

//*****************************************************************************
// TASK			: OSEK_Task_ts3
// ARGUMENT		: none
// RETURN		: none
// DESCRIPTION 	: 100msec periodical Task for bluetooth communication
//*****************************************************************************
TASK(OSEK_Task_ts3)
{
	static U8 buffer[BT_RCV_BUF_SIZE]; /* Bluetooth receive buffer(32bytes) */

	U32 length = ecrobot_read_bt_packet(buffer, BT_RCV_BUF_SIZE);
	/*
	 * R/C command from NXT GamePad
	 * buf[0]: -100(forward max.) to 100(backward max.)
	 * buf[1]: -100(turn left max.) to 100(turn right max.)
	 */
	if (length > 0) // if we receive something then update values
	{
		U8 command = buffer[0];
		if (command == 1)
		{
			state.targetforward = (S8) buffer[1]; /* reverse the direction */
			state.targetturn = (S8) buffer[2];
			state.update = 1;
		}
		else if (command == 2)
		{
			U8 newbehavior = buffer[1];
			J5BehaviorFunction = J5BehaviorArray[newbehavior];
		}
	}
	//ecrobot_show_int((S32)state.forward);

	TerminateTask(); /* terminates this task and executed by System Timer Count */
}

//*****************************************************************************
// TASK			: OSEK_Task_Background
// ARGUMENT		: none
// RETURN		: none
// DESCRIPTION 	: Background(never terminated) Task
//*****************************************************************************
TASK(OSEK_Task_Background)
{
	while (1)
	{
		ecrobot_status_monitor("J5"); /* LCD display */

		systick_wait_ms(500U); /* 500msec wait */
	}
}

/*
 * volatile int BTResetCounter=0;
 void resetBlueTooth(void)
 {
 ecrobot_term_bt_connection();
 ecrobot_init_bt_slave(BT_PASS_KEY);
 }
 BTResetCounter++;
 if(BTResetCounter>5*10) // every 5 seconds the bluetooth connection is reset.
 {
 BTResetCounter=0;
 resetBlueTooth();
 }
 */

/******************************** END OF FILE ********************************/

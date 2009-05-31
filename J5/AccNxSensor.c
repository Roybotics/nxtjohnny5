/**
 *******************************************************************************
 **	FILE NAME : AccNxSensor.c
 **
 **	ABSTRACT  : Mindsensors accelerometer driver.
 ** Acknowledgments: based on code from Takashi Chikamasa.
 *******************************************************************************
 **/


#include "math.h"
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"



/*
 * Initialize the specified port to be used for I2C communication
 * NOTE: user defined I2C sensor initialize function should be implemented
 *       in user defined an OSEK initialization Task (not in LEJOS OSEK device init hook).
 *       because device init hook is invoked in a loop while the button instruction screen is appeared.
 */
void init_mindsensor_accel_sensor(U8 port_id)
{
	nxt_avr_set_input_power(port_id,2);
	i2c_enable(port_id);
}

/*
 * This is an implementation example of I2C sensor data acquisition for
 * mindsensor acceleration sensor. mindsensor acceleration sensor returns
 * tilt data and acceleration data in three axes.
 *
 * This API implementation for I2C communication might be different from
 * I2C sensor communication examples in other NXT programming languages.
 * Others use a wait until data transaction is finished after sending a request.
 * However, it might not be acceptable for real-time control application. So we
 * introduce one sampling delay to avoid waiting for the completion of the data acqusition.
 */
void get_mindsensor_accel_sensor(U8 port_id, S16 *buf)
{
	 int i,j;
	 static S16 tilt_state[3];
	 static S16 accel_state[3];
	 static U8 data[9] = {0,0,0,0,0,0,0,0,0};
	/*
	 * Data spec. of mindsensor acceleration sensor
	 *
	 * 0x42 data[0]: X axis Tilt data
	 * 0x43 data[1]: Y axis Tilt data
	 * 0x44 data[2]: Z axis Tilt data
	 * 0x45 data[3]: X axis Accel LSB
	 * 0x46 data[4]: X axis Accel MSB
	 * 0x47 data[5]: Y axis Accel LSB
	 * 0x48 data[6]: Y axis Accel MSB
	 * 0x49 data[7]: Z axis Accel LSB
	 * 0x4A data[8]: Z axis Accel MSB
	 */

	if (i2c_busy(port_id) == 0) /* check the status of I2C comm. */
	{
		j=0;
		for (i=0; i<8; i++)
		{
			if (i<3)
			{
				tilt_state[i] = (S16)data[i];
			}
			else
			{
				accel_state[j++] = ((S16)data[i+1]<<8) + (S16)data[i];
				i++;
			}
   		}
	   /* i2c_start_transaction just triggers an I2C transaction,
		* actual data transaction between ARM7 and an Acceleration
		* Sensor is done by an ISR after this, so there is one execution cycle
		* delay for consistent data acquistion
		*/
		i2c_start_transaction(port_id,1,0x42,1,data,9,0);
	}

	for (i=0; i<3; i++)
	{
		buf[i] = tilt_state[i];
		buf[i+3] = accel_state[i];
   	}
}

F32 getaccelnorm(S16 *buf)
{
	F32 norm =0;
	norm += ((F32)buf[3])*((F32)buf[3]);
	norm += ((F32)buf[4])*((F32)buf[4]);
	norm += ((F32)buf[5])*((F32)buf[5]);
	return sqrt(norm)/1000000000;
}

/*
 * Terminate I2C communication on the specified port
 */
void term_mindsensor_accel_sensor(U8 port_id)
{
	i2c_disable(port_id);
}

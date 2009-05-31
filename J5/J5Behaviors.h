/**
 ******************************************************************************
 **	FILE NAME : J5Behaviors.c
 **
 **	ABSTRACT  : J5 (Johnny 5) Behaviors header file
 **
 *******************************************************************************
 **/

typedef struct
{
	U32 gyro;
	float gyro_offset;

	S16 accl[6];
	float acclnorm;
	float acclavg;
	float accldelta;

	S16 sonar;

	U8* camdata; //length:41

	S8 update;
	S8 targetforward;
	S8 targetturn;

	int seektargetheadangle;
	int targetheadangle;
	int headangle;

	float forward;
	float turn;

	int motorcountL;
	int motorcountR;
	int motorcountH;
	S8 pwmL;
	S8 pwmR;
	S16 pwmLR;
	S8 pwmH;
} State;

void head_control(State* state);

void standstillbehavior(State* state);
void stablestandstillbehavior(State* state);

void trackredball(State* state);

/******************************** END OF FILE ********************************/

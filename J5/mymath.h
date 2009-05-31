/**
 ******************************************************************************
 **	FILE NAME : mymath,h
 **
 **	ABSTRACT  : Math functions and macros.
 *******************************************************************************
 **/

#include "math.h"

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
#define ABS(X) ((X) < 0 ? -(X) : (X))
#define SIGN(X) ( ((X)>0) - ((X)<0) )
#define CLAMP(X,MIN,MAX) ( ((X)>(MAX)) ? (MAX) : ( ((X)<(MIN)) ? (MIN) : (X) ) )
#define ADDSTEP(X,STEP) ( ((X)>0) ? ((X)+(STEP)) : ( ((X)<0) ? ((X)-(STEP)) : (X) ) )
#define PI (3.14159265358979323846)

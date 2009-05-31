# Target specific macros
TARGET = J5

NXTOSEK_ROOT = ../..

# nxtway_gs_balancer library desiged for NXTway-GS two wheeled self-balancing robot
USER_INC_PATH= $(NXTOSEK_ROOT)/ecrobot/nxtway_gs_balancer
USER_LIB = nxtway_gs_balancer

# using NXT standard tires (not Motorcycle tires)
#USER_DEF = NXT_STD_TIRE

# User application source
TARGET_SOURCES := \
	balancer_param.c \
	J5.c \
	J5Behaviors.c\
	AccNxSensor.c\
	PID.c\
	NxtCam.c

# OSEK OIL file
TOPPERS_OSEK_OIL_SOURCE := ./J5.oil

# below part should not be modified
O_PATH ?= build
include $(NXTOSEK_ROOT)/ecrobot/ecrobot.mak

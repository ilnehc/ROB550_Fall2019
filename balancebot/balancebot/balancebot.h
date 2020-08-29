#ifndef BB_H
#define BB_H

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h> // for isatty()
#include <string.h>
#include <math.h> // for M_PI
#include <signal.h>
#include <pthread.h>
#include <rc/mpu.h>
#include <rc/math/quaternion.h>
#include "../common/mb_defs.h"
#include "../common/mb_structs.h"
#include "../common/mb_motor.h"
#include "../common/mb_controller.h"
#include "../common/mb_odometry.h"
#include "../xbee_serial/xbee_receive.h"

// # Define for Manual Control

#define DRIVE_RATE_NOVICE	23
#define TURN_RATE_NOVICE	4
#define DRIVE_RATE_ADVANCED	26
#define TURN_RATE_ADVANCED	10

// DSM channel config
#define DSM_DRIVE_POL		1
#define DSM_TURN_POL		1
#define DSM_DRIVE_CH		3 
#define DSM_TURN_CH		4
#define DSM_DEAD_ZONE		0.08
#define STEERING_INPUT_MAX 1


// global variables
rc_mpu_data_t mpu_data;

//drive_mode_t drive_mode_t;
//m_intput_mode_t m_intput_mode_t;

pthread_mutex_t state_mutex;
pthread_mutex_t setpoint_mutex;
mb_state_t mb_state;
mb_setpoints_t mb_setpoints;
mb_odometry_t mb_odometry;

xbee_packet_t xbeeMsg;
int XBEE_portID;

// functions
int64_t utime_now();
float clamp_gamma_setpoint(float angle);
void balancebot_controller();


//threads
void* setpoint_control_loop(void* ptr);
void* printf_loop(void* ptr);

#endif

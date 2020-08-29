#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H


#include "mb_structs.h"
#define CFG_PATH "pid.cfg"
#define Max_Motor_Torque 	2 // (In Nm)

int mb_controller_init(float* gains);
int mb_controller_load_config(float* gains);
int mb_controller_update(mb_state_t* mb_state);
int mb_controller_cleanup();

#endif


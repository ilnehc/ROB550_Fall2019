/*******************************************************************************
* measure_motors.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the parameters for your motors
*
* TODO: Capture encoder readings, current readings, timestamps etc. to a file
*       to analyze and determine motor parameters
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include "../common/mb_motor.h"

FILE* f1;

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	
	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
        return -1;
    }

	// initialize enocders
    if(rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();


    rc_set_state(RUNNING);
    while(rc_get_state()!=EXITING){
    	rc_nanosleep(1E9);
    }

	// exit cleanly
    rc_adc_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}

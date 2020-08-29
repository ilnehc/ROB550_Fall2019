#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"
#include <stdlib.h>

/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_controller_init(float* gains){

    mb_controller_load_config(gains);
    /* TODO initialize your controllers here*/

    /*
    // Creating an empty initialization of the filter
    rc_filter_t D1=RC_FILTER_INITIALIZER;
    rc_filter_t D2=RC_FILTER_INITIALIZER;

    // Setting up the controllers

    // SAMPLE_RATE_HZ IS NOT DEFINED
    if(rc_filter_pid(&D1, gains[0], gains[1], gains[2], 4*SAMPLE_RATE_HZ, SAMPLE_RATE_HZ)){
        fprintf(stderr,"ERROR: Controller for Inner Loop D1 could not be made");   
    }

    if(rc_filter_pid(&D2, gains[3], gains[4], gains[5], 4*SAMPLE_RATE_HZ, SAMPLE_RATE_HZ)){
        fprintf(stderr,"ERROR: Controller for Outer Loop D2 could not be made");   
    }

    // Setting up the saturation on the controllers

    // Inner loop controller ouputs desired torque (Max torque is limited by motor output)
    rc_filter_enable_saturation(&D1, -Max_Motor_Torque, Max_Motor_Torque)

    // No saturation needed for outer controller


    // if(rc_filter_alloc_from_arrays(&D1, DT, D1_num, D1_num_len, D1_den, D1_den_len)){
    //     fprintf(stderr,"ERROR: Controller for Inner Loop D1 could not be made");
    // }


    // if(rc_filter_alloc_from_arrays(&D2, DT, D2_num, D2_num_len, D2_den, D2_den_len)){
    //     fprintf(stderr,"ERROR: Controller for Outer Loop D2 could not be made");
    // }

    */

    return 0;
}

/*******************************************************************************
* int mb_controller_load_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_controller_load_config(float* gains){
    /*
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening %s\n", CFG_PATH );
    }

    float gains[6];
    for (int i=0;i<6;i++){
        fscanf(file, "%f",&gains[i]);
    }
    fclose(file);
    return 0;
    */

    int ELEMENTS = 12;

    FILE *myfile;
    double myvariable;
    int i;
    int j;
    int idx = 0;
    myfile = fopen("/home/debian/balancebot-f19/common/PID_gains.txt", "r");

    if(myfile == NULL){
        printf("file didn't get opened\n");
    }

    for(i = 0; i < ELEMENTS; i++)
    {
        fscanf(myfile,"%lf",&myvariable);
        //printf("%.4f ",myvariable);
        gains[idx] = myvariable;
        //printf("gain %d: %f\n", idx, gains[idx]);
        idx++;
        //printf("\n");
    }

    fclose(myfile);
    return 0;

}

/*******************************************************************************
* int mb_controller_update()
* 
* 
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* this should only be called in the imu call back function, no mutex needed
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state){
    /*TODO: Write your controller here*/
    return 0;
}


/*******************************************************************************
* int mb_controller_cleanup()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_cleanup(){
    // Releasing the memory resources taken up by PID controller
    //rc_filter_free(&D1);
    //rc_filter_free(&D2);
    return 0;
}
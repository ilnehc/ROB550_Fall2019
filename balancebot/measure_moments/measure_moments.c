/*******************************************************************************
* measure_moments.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the moments of inertia of your Balancebot
* 
* TODO: capture the gyro data and timestamps to a file to determine the period.
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
#include <rc/mpu.h>

//#define I2C_BUS 2

FILE* f1;

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	
    // Defining the filestream	
    FILE* fp=fopen("gyro_data.csv", "r");

    if(fp==NULL){
	printf("File does not exist\n");
	printf("Creating the file to save the data\n");
	fp=fopen("gyro_data.csv","wb");
    }
    else{
	printf("File Opened for Writing data\n");
    }

    int ret=rc_kill_existing_process(1.5);

    switch(ret){

	case 0:
		printf("No existing process running\n");
		break;
	case 1:
		printf("Existing process was running\n");
		break;
	default:
		break;			
	}	    
    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	//rc_make_pid_file();


    //rc_set_state(RUNNING);

    // Initializing the IMU Gyro
    // Creating Instance of rc_mpu_data_t class
    rc_mpu_data_t data;

    // Setting the gyro data to degree mode
    // g_mode_t g_mode=G_MODE_DEG;
    // Setting the accelerometer data to m/s^2
    // a_mode_t a_mode=A_MODE_MS2;

    // Creating the data configuration file instance
    rc_mpu_config_t conf=rc_mpu_default_config();
    //  conf.i2c_bus=I2C_BUS;

    // Checking if intialization is done correctly
    if(rc_mpu_initialize(&data, conf)){
	fprintf(stderr,"rc_mpu_initialize FAILED\n");
    }
    else{
	printf("Initialization Complete\n");
    }
    fprintf(fp,"Accel XYZ (m/s^2) |");
    fprintf(fp,"Gyro  XYZ (deg/sec)  |");
    fprintf(fp,"Time (nanoseconds)\n");


    // Creating the variables for recording the time
    uint64_t time_start,time_end;

    time_start=rc_nanos_since_epoch();

    while(1){

    	// Command to refresh the screen
	//printf("\r");

    	if(rc_mpu_read_accel(&data)<0){
    		printf("Reading acclerometer data failed\n");
    	}

    	if(rc_mpu_read_gyro(&data)<0){
		printf("Reading gyro data failed\n");
    	}    

    	// Now Printing all data
	
    	fprintf(fp,"%.3f %.3f %.3f ", data.accel[0],\
			              data.accel[1],\
				      data.accel[2]);

	fprintf(fp,"%.3f %.3f %.3f ", data.gyro[0],\
			              data.gyro[1],\
			 	      data.gyro[2]);
	
	time_end=rc_nanos_since_epoch()-time_start;

	//int time_diff=(int)(time_end-time_start);

	long long int time_int=(long long)(time_end);

	fprintf(fp,"%lld",time_int);
	fprintf(fp,"\n");
	rc_usleep(200);
    }

    fprintf(fp,"\n");

    // Powering off the MPU
    rc_mpu_power_off();


   // exit cleanly
   //rc_encoder_eqep_cleanup(); 
   //rc_remove_pid_file();   // remove pid file LAST
   return 0;
}

/*******************************************************************************
* balancebot.c
*
* Main template code for the BalanceBot Project
* based on rc_balance
* 
*******************************************************************************/

#include <math.h>
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <rc/math.h>
#include <sys/time.h>
#include <stdlib.h>


#include "balancebot.h"

rc_filter_t D1 = RC_FILTER_INITIALIZER;	//P Control for inner loop
rc_filter_t D3 = RC_FILTER_INITIALIZER;	//PD Control for outer loop
rc_filter_t Inner_I = RC_FILTER_INITIALIZER; //I Control for inner loop
rc_filter_t steering = RC_FILTER_INITIALIZER; // Steering Controller

#define Tf	0.05
#define TF_outer 0.194
#define TF_st 0.05

//#define	theta_offset	0

float INNER_LOOP_GAIN = 2.0;
float D1_KP = -1.5750;		//Inner loop PD
float D1_KI = -9.000;
float D1_KD = -0.0720;

#define D2_SATURATION_TIMEOUT	0.4

float OUTER_LOOP_GAIN = 1.0;
float D3_KP	= 0.0050;		//Outer loop PD control
float D3_KI	= 0.0005;
float D3_KD	= 0.0110;

float STEERING_LOOP_GAIN = 1.0;
float st_KP =  1.5; //0.050;
float st_KI = 0;
float st_KD = 0;

float gammadot_setpoint=0;

#define TIP_ANGLE		0.52
#define ENABLE_POSITION_HOLD    	0
#define ENABLE_STEERING			1
#define ENABLE_ODOMETRY			1
#define MAX_THETA	0.5
#define ENABLE_SQUARE			1
#define ENABLE_DRAG_RACE		1
#define ENABLE_DATA_COLLECTION		1

// time tracking variable
int64_t current_time = 0;
float* gains;

/*******************************************************************************
* int main() 
*******************************************************************************/
int main(){
	gains = (float*)malloc(sizeof(float)*12); 

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
        fprintf(stderr,"Failed to set governor to PERFORMANCEDT\n");
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

    if(rc_dsm_init()==-1){
		fprintf(stderr,"failed to start initialize DSM\n");
		return -1;
	}

	printf("initializing xbee... \n");
	//initialize XBee Radio
	int baudrate = BAUDRATE;
	if(XBEE_init(baudrate)==-1){
		fprintf(stderr,"Error initializing XBee\n");
		return -1;
	};

	// Setting the drive mode and input mode
	//m_input_mode_t m_input_mode = DSM;
	//drive_mode_t drive_input_mode=NOVICE;
	mb_setpoints.drive_mode_t=NOVICE;

	//read in gains from file
	mb_controller_load_config(gains);
	//loop read gains
	INNER_LOOP_GAIN = gains[0];
	D1_KP = gains[1];
	D1_KI = gains[2];
	D1_KD = gains[3];
	OUTER_LOOP_GAIN = gains[4];
	D3_KP = gains[5];
	D3_KI = gains[6];
	D3_KD = gains[7];
	STEERING_LOOP_GAIN = gains[8];
	st_KP = gains[9];
	st_KI = gains[10];
	st_KD = gains[11];


	//initialize Inner loop PID controller
	if(rc_filter_pid(&D1, D1_KP, D1_KI, D1_KD, Tf, DT)){
		fprintf(stderr,"ERROR: Pd Controller for Inner Loop D1 could not be made\n");
		return -1;
	}
	rc_filter_enable_saturation(&D1, -1.0, 1.0);

	//outer loop
	if(rc_filter_pid(&D3, D3_KP, D3_KI, D3_KD, TF_outer, DT)){
		fprintf(stderr,"ERROR: PID Controller for Outer Loop D3 could not be made\n");
		return -1;
	}

	if(rc_filter_pid(&steering, st_KP, st_KI, st_KD, TF_st, DT)){
		fprintf(stderr,"ERROR: PID Controller for Steering could not be made\n");
		return -1;
	}

	rc_filter_enable_saturation(&steering, -STEERING_INPUT_MAX, STEERING_INPUT_MAX);

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	printf("starting print thread... \n");
	pthread_t  printf_thread;
	rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void*) NULL, SCHED_FIFO, 50);

	mb_setpoints.phi=0;
	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_DOWN;

	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	//rc_mpu_calibrate_gyro_routine(mpu_config);

	//rc_nanosleep(5E9); // wait for imu to stabilize

	//initialize state mutex
    	pthread_mutex_init(&state_mutex, NULL);
    	pthread_mutex_init(&setpoint_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_controller_init(gains);

	printf("initializing motors...\n");
	mb_motor_init();
	mb_motor_brake(1);

	printf("resetting encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);

	printf("initializing odometry...\n");
	mb_odometry_init(&mb_odometry, 0.0,0.0,0.0);

	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&balancebot_controller);

	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threadsint inner_saturation_counter=0;
		// there is no need to do anything here but sleep
		// always sleep at some point		
		rc_nanosleep(1E9);
	}
	
	// exit cleanly
	rc_filter_free(&D1);
	rc_filter_free(&D3);
	rc_filter_free(&steering);
	rc_mpu_power_off();
	mb_motor_cleanup();
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file(); // remove pid file LAST 
	return 0;
}


/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
* 
*
*******************************************************************************/
void balancebot_controller(){

	/*int64_t time_now = utime_now();

	if((time_now - current_time)% (1*10^6) > 0){ 
	
		current_time = time_now;
		mb_controller_load_config(gains);
		printf("gain 1: %f", gains[0]);
		//loop read gains
		D1_KP = gains[0];
		D1_KI = gains[1];
		D1_KD = gains[2];
		D3_KP = gains[3];
		D3_KI = gains[4];
		D3_KD = gains[5];

		//printf("Gains are %lf, %lf, %lf\n",D1_KP,D1_KI,D1_KD);
	}
	*/

	//lock state mutex
	pthread_mutex_lock(&state_mutex);

	// old way of finding values
	// Read IMU
	
	// Read encoders
	
    // Calculate controller outputs
    // Phi is average wheel rotation also add theta body angle to get absolute
	// wheel position in global frame since encoders are attached to the body
	//cstate.phi = ((cstate.wheelAngleL+cstate.wheelAngleR)/2) + cstate.theta;


	if (!ENABLE_ODOMETRY){
		mb_state.left_encoder = (rc_encoder_eqep_read(1)*2.0*M_PI)/(ENC_1_POL*GEAR_RATIO*ENCODER_RES);
		mb_state.right_encoder = (rc_encoder_eqep_read(2)*2.0*M_PI)/(ENC_2_POL*GEAR_RATIO*ENCODER_RES);
		mb_state.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X];
		mb_state.phi = ((mb_state.left_encoder + mb_state.right_encoder)/2) + mb_state.theta; //angle traveled
		mb_state.gamma = (mb_state.left_encoder-mb_state.right_encoder)*((WHEEL_DIAMETER/2)/WHEEL_BASE); // body orientation //get from odometry
		mb_odometry.gyro_gamma = mpu_data.dmp_TaitBryan[TB_YAW_Z];
		//printf("Set point gamma is %f",mb_setpoints.gamma);
	}

	else{
	//calc values with the odometry
		mb_odometry_update(&mb_odometry, &mb_state);
	}

	// check for a tipover
	// if(fabs(mb_state.theta) > TIP_ANGLE){
	// 	mb_motor_disable();
	// 	printf("tip detected \n");
	// 	return;
	// }

	/************************************************************
	* OUTER LOOP PHI controller D2
	* Move the position setpoint based on phi_dot.
	* Input to the controller is phi error (setpoint-state).
	*************************************************************/
	// mb_setpoints.phi = 0;
	// mb_setpoints.phi_dot = 0;
	// mb_setpoints.gamma = 0.0;

	//printf("Current phi position is %lf",mb_setpoints.phi);

	if(ENABLE_POSITION_HOLD){
		if(fabs(mb_setpoints.phi_dot) > 0.001) 
		mb_setpoints.phi += mb_setpoints.phi_dot*DT;//velocity*time + current_position = new position in relation to current position
		float pd_output_outer = rc_filter_march(&D3,mb_setpoints.phi-mb_state.phi); //(position we want to be - position we are at)
		float pid_outer_bodyangular = OUTER_LOOP_GAIN * (pd_output_outer);
		mb_setpoints.theta = pid_outer_bodyangular; //body move angle
	}
	//else 
	//	mb_setpoints.theta = 0.00; //body balance angel

	/************************************************************
	* INNER LOOP ANGLE Theta controller + i_output + d_output
	* Input to D1 is theta error (setpoint-state).
	*************************************************************/

	float p_output = rc_filter_march(&D1,(mb_setpoints.theta - mb_state.theta));
	float pid_output_pwm = INNER_LOOP_GAIN * (p_output);


	mb_state.left_cmd = pid_output_pwm;
	mb_state.right_cmd = pid_output_pwm;

	/************************************************************
	* STEERING ANGLE Gamma controller 
	* Input to Steering Controller is Gamma error (setpoint-state).
	*************************************************************/
		
		

	if(fabs(mb_setpoints.gamma_dot)>0.0001)
        gammadot_setpoint += mb_setpoints.gamma_dot * DT;// + mb_state.gamma; // desired orientation
	

	//mb_setpoints.gamma=gammadot_setpoint;
	//mb_setpoints.gamma=clamp_gamma_setpoint(gammadot_setpoint);
	
	
	//calculate error term for body angle
	float error1 = mb_setpoints.gamma - mb_state.gamma;
	if(error1 > 177)
		error1 -= 0.0;
	if(error1 < -177)
		error1 -= 0.0;
	//error1 = clamp_gamma_setpoint(error1);
	float error2 = error1 -(float)(2*M_PI);
        float error3 = error1 + (float)(2*M_PI);
	float best_error;
	if(fabs(error1)<fabs(error2)){
		if(fabs(error1)<fabs(error3)){
			best_error = error1;
		}
		else{
			best_error = error3;
		}
	}
	else{
		if(fabs(error2)<fabs(error3)){
			best_error = error2;
		}
		else{
			best_error = error3;
		}
	}


	//body angle filter
	float st_PID_output = rc_filter_march(&steering,best_error);//(desired orientation - current orientation)
				
	

	if (!ENABLE_STEERING)
		st_PID_output=0;
	/*************************************************************
	* Check if the inner loop saturated. If it saturates for over
	* a second disarm the controller to prevent stalling motors.
	************************************************************int inner_saturation_counter=0;*/
	// int inner_saturation_counter=0;
	//int inner_saturation_counter=0;
	/**********************************************************int inner_saturation_counter=0;
	* Send signal to motors
	* add D1 balance control u and D3 steering control also
	* multiply by polarity to make sure direction is correct.
	***********************************************************/
	double dutyL = mb_state.left_cmd + st_PID_output;
	double dutyR = mb_state.right_cmd - st_PID_output;
	
	double duty_check(double duty){
		if(duty>1)
			duty=1;
		else if (duty <-1)
			duty=-1;
	return duty;
	}

	dutyL=duty_check(dutyL);
	dutyR=duty_check(dutyR);
	
	//printf("duty: %f\n", dutyL);
	//rc_motor_set(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * dutyL);
	//rc_motor_set(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * dutyR);

	mb_motor_set(LEFT_MOTOR, dutyL*MOT_1_POL*-1);
	mb_motor_set(RIGHT_MOTOR, dutyR*MOT_2_POL*-1);

    
    if(!mb_setpoints.manual_ctl){
    	//send motor commands
   	}

    if(mb_setpoints.manual_ctl){
    	//send motor commands
   	}

	// XBEE_getData();
	// double q_array[4] = {xbeeMsg.qw, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz};
	// double tb_array[3] = {0, 0, 0};
	// rc_quaternion_to_tb_array(q_array, tb_array);
	// mb_state.opti_x = xbeeMsg.x;
	// mb_state.opti_y = -xbeeMsg.y;	    //xBee quaternion is in Z-down, need Z-up
	// mb_state.opti_roll = tb_array[0];
	// mb_state.opti_pitch = -tb_array[1]; //xBee quaternion is in Z-down, need Z-up
	// mb_state.opti_yaw = -tb_array[2];   //xBee quaternion is in Z-down, need Z-up
	

   	//unlock state mutex
    pthread_mutex_unlock(&state_mutex);

}

float clamp_gamma_setpoint(float angle){
	float pi = (float)(M_PI);
	if (angle<-1*pi){
		angle=angle+(2*pi);	
	}
	else if(angle>pi){
		angle=angle-(2*pi);	
	}
	return angle;
}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){
	
    // wait for mpu to settle
    //__disarm_controller();
    rc_usleep(2500000);
    rc_set_state(RUNNING);
	mb_setpoints.phi=0;
	mb_setpoints.gamma=0;
	float ref_distance=(0.95)/(WHEEL_DIAMETER/2);
	int correction_angle=0;
	int count=0;
	float increment_val=1.06;
	int n = 1;
	float new_angle = 0;
	float diff_val = 999;
	float square_heading = 0;
	float gamma_matrix[4]={0,M_PI/2,M_PI,-M_PI/2};
	int run_control_loop = 1;
	if(!run_control_loop){
		return NULL;
	}

	while(1){
		
		if(rc_dsm_is_new_data()){
		
			// TODO: Handle the DSM data from the Spektrum radio reciever
			// You may should implement switching between manual and autonomous mode
			// using channel 5 of the DSM data.

				

			if (rc_dsm_ch_normalized(5)>0){			
				//mb_setpoints.theta=0.03;
				//mb_setpoints.phi=0.03/(WHEEL_DIAMETER/2);
				mb_setpoints.gamma=(float)(0.0*M_PI/180.0);
				mb_setpoints.gamma_dot = 0.0;

				/*
				printf("DOING DRAG RACE\n");
				
				mb_setpoints.theta = 4.0*M_PI/180;
				if(mb_setpoints.phi>=(11.0/(WHEEL_DIAMETER/2)))
				{
					increment_val=0;
					mb_setpoints.theta = 0;
				}

				if(mb_setpoints.phi>=(10.6/(WHEEL_DIAMETER/2)) && mb_setpoints.phi<(11.0/(WHEEL_DIAMETER/2)))
				{
					increment_val=0.3;
				}
					
				mb_setpoints.phi+=increment_val;
				*/		
			}
			
			else if (rc_dsm_ch_normalized(7)>0){
				
				//mb_setpoints.gamma=(-180*M_PI/180);
				//mb_setpoints.gamma_dot = 0.0;
				
				/*while(mb_setpoints.phi<0.3/(WHEEL_DIAMETER/2)){
					rc_nanosleep(0.5E8);
					mb_setpoints.phi+=0.2;
				}
				
				*/
				
				//printf("INSIDE THE LOOP\n");
				
				/*
				while(mb_setpoints.theta<0.05)
				{
					rc_nanosleep(1E9);
					mb_setpoints.theta+=0.005;
				}
				
				*/
						
				mb_setpoints.theta=0.0;
				
				printf("DOING SQUARE TRAJECTORY\n");
				/*

				mb_setpoints.phi+=0.05;
				
				//if (mb_setpoints.phi>=(4*1.5/(WHEEL_DIAMETER/2)))
				//	mb_setpoints.phi=4*1.5/(WHEEL_DIAMETER/2);
				

				if (mb_state.phi>=(ref_distance-0.3)){
					//printf("GAMMA SET to -90\n");
					//square_heading=((-correction_angle)*M_PI/180);
						
					//mb_setpoints.gamma+=(88*M_PI)/180+square_heading;
					new_angle = gamma_matrix[n];
					float diff_val1 = mb_state.gamma - new_angle;
					float diff_val2 = diff_val1 + (float)(2*M_PI);
					float diff_val3 = diff_val1 - (float)(2*M_PI);
					
					if(fabs(diff_val1) < fabs(diff_val2)){
						diff_val = fabs(diff_val1);
					}
					else{
						diff_val = fabs(diff_val2);
					}

					if(abs(diff_val3) < diff_val){
						diff_val = fabs(diff_val3);
					}
					
					//printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f -------------------------------------------\n",diff_val1, diff_val2, diff_val3, diff_val, mb_state.gamma, new_angle);
					while(diff_val > 0.07 ){
						//mb_motor_set(1,0.05);	
						mb_setpoints.gamma += 2*M_PI/180;
						rc_nanosleep(2E8);
						//count++;
						//if(mb_state.gamma==(n*M_PI/2))
						//	mb_setpoints.gamma+=square_heading;
					
					diff_val1 = mb_state.gamma - new_angle;
					diff_val2 = diff_val1 + (float)(2*M_PI);
					diff_val3 = diff_val1 - (float)(2*M_PI);
	
					if(fabs(diff_val1) < fabs(diff_val2)){
						diff_val = fabs(diff_val1);
					}
					else{
						diff_val = fabs(diff_val2);
					}

					if(fabs(diff_val3) < diff_val){
						diff_val = fabs(diff_val3);
					
					}

					//printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %d, -------------------------------------------\n",diff_val1, diff_val2, diff_val3, diff_val, mb_state.gamma, new_angle, n);
					}
					mb_setpoints.gamma = new_angle;
					ref_distance+=(0.95)/(WHEEL_DIAMETER/2);
					n+=1;	
					if(n == 4){
						//printf("n=4\n");
						n = 0;
					}	
					//printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %d, -------------------------------------------\n",diff_val1, diff_val2, diff_val3, diff_val, mb_state.gamma, new_angle, n);
				
				}*/
				
			}
				
				
			while(0){
				// Read normalized (+-1) inputs from RC radio stick and multiply by
				// polarity setting so positive stick means positive setpoint
				
				//printf("New Data Being Received\n");
				double turn_stick  = (rc_dsm_ch_normalized(DSM_TURN_CH)) * DSM_TURN_POL*-1;//-0.3
				double drive_stick = (rc_dsm_ch_normalized(DSM_DRIVE_CH))* DSM_DRIVE_POL;//+0.28 

				//printf("Phi and theta are %lf and %lf\n",mb_setpoints.phi,mb_setpoints.theta);
				//printf("Setpoint Gamma is %lf \n",mb_setpoints.gamma);
				//printf("Current Gamma is %lf \n",mb_state.gamma);


				// saturate the inputs to avoid possible erratic behavior
				rc_saturate_double(&drive_stick,-1,1);
				rc_saturate_double(&turn_stick,-1,1);
				// use a small deadzone to prevent slow drifts in position
				if(fabs(drive_stick)<DSM_DEAD_ZONE)
						drive_stick = 0.0;
				if(fabs(turn_stick)<DSM_DEAD_ZONE)  
						turn_stick  = 0.0;

				//printf("Turn Stick and Drive Stick %lf and  %lf\n",turn_stick,drive_stick); 
				// translate normalized user input to real setpoint values
				mb_setpoints.phi_dot=DRIVE_RATE_NOVICE*drive_stick;
				mb_setpoints.gamma_dot=TURN_RATE_NOVICE*turn_stick;
				}
				
		}

	    // if dsm had timed out, put setpoint rates back to 0
	    else if(rc_dsm_is_connection_active()==0){
			 	printf("DSM Connection Not Active\n");
			 	mb_setpoints.theta = 0;
			 	mb_setpoints.phi_dot = 0;
			 	mb_setpoints.gamma_dot = 0;
			 	// mb_setpoints.gamma=-M_PI/2;
			 	return NULL;
	    }
		rc_nanosleep(1E9 / RC_CTL_HZ);
		
	}
	return NULL;
}




/******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state
	
	uint64_t time_start, time_end;

	time_start=rc_nanos_since_epoch();

	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                 SENSORS               |            MOCAP            |");
			printf("\n");
			printf("    θ    |");
			printf("    φ    |");
			printf("  Time   |");
			printf("  Gamma  |");
			
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    ψ    |");
			printf(" Sat Flag|");
		
			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if(new_state == RUNNING){
			//printf("\r");
			// Add Print stattements here, do not follow with /n
			pthread_mutex_lock(&state_mutex);
			printf("%7.3f  |", mb_state.theta);
			//printf("%7.3f  |", mb_state.phi);
			time_end=rc_nanos_since_epoch()-time_start;
			long long int time_int=(long long)(time_end);
			printf("%lld   |\n", time_int);		
			//printf("%lld,", time_int);
			//printf("%.3f  |", mb_state.gamma);
			//printf("%.3f,", mb_state.gamma);
			//printf("%.3f,", mb_state.phi);
			//printf("%7.3f,", mb_odometry.x);
			//printf("%7.3f,\n", mb_odometry.y);
			/*
			printf("%7.3f  |", mb_state.left_encoder);
			printf("%7.3f  |", mb_state.right_encoder);
			printf("%7.3f  |", mb_odometry.x);
			printf("%7.3f  |", mb_odometry.y);
			printf("%7.3f  |", mb_odometry.gyro_gamma);
			printf("%7d  |\n", mb_state.mb_saturated);
			*/

			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);
	}
	return NULL;
} 

int64_t utime_now(){
	struct timeval tv;
	gettimeofday (&tv, NULL);
	return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

		

/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"
#include <rc/mpu.h>

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float gamma){
/* TODO */
mb_odometry->x = x;
mb_odometry->y = y;
mb_odometry->gamma = gamma; // heading
mb_odometry->right_encoder =  rc_encoder_eqep_read(RIGHT_MOTOR); // encoder value for left wheel
mb_odometry->left_encoder = rc_encoder_eqep_read(LEFT_MOTOR); // encoder value for right wheel
mb_odometry->gyro_gamma = mpu_data.gyro[2]; // z-axis heading value from gyro
mb_odometry->phi_odo=0.0;
gamma_offset = 0; //set to gamma

}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
/* TODO */
//robot dimensions
float wheel_radius = WHEEL_DIAMETER / 2.0; // wheel radius (m)
//WHEEL_BASE // distance between wheels (m)
//ENCODER_RES // encoder resolution (transitions per motor shaft revolution)
//GEAR_RATIO // gear ration of motors

//stored previous state variables
prev_x = mb_odometry->x; // previously known x position (m)
prev_y = mb_odometry->y; // previously known y position (m)
prev_gamma = mb_odometry->gamma; // previously known body angle (rad)
prev_Rw_enc = mb_odometry->right_encoder; // previous value of right wheel encoder
prev_Lw_enc = mb_odometry->left_encoder;  // previous value of left wheel encoder

//new sensor data since last update
int Lw_enc = rc_encoder_eqep_read(LEFT_MOTOR); // encoder value for left wheel
int Rw_enc = rc_encoder_eqep_read(RIGHT_MOTOR); // encoder value for right wheel

//calc theta, phi, gamma (using encoder data for gamma)
float theta = mpu_data.dmp_TaitBryan[TB_PITCH_X]; //body angle about wheel axis
float Lw_phi = (Lw_enc*2.0*M_PI)/(ENC_1_POL*GEAR_RATIO*ENCODER_RES);
float Rw_phi = (Rw_enc*2.0*M_PI)/(ENC_2_POL*GEAR_RATIO*ENCODER_RES);
float phi =  ((Lw_phi + Rw_phi)/2) + theta; //angle traveled by the wheels
float gamma = (Lw_phi-Rw_phi)*((WHEEL_DIAMETER/2.0)/WHEEL_BASE); // body orientation
//printf("gamma: %f\n", gamma);

//Dead Reckoning -----------------------------------------------------------
//gyro heading given by IMU
if(rc_mpu_read_gyro(&mpu_data)<0){
	printf("Reading gyro data failed\n");
}  
//float gyro_heading = mpu_data.gyro[2]*DEG_TO_RAD;  //orientation about z-axis
float gyro_heading = -1*mpu_data.dmp_TaitBryan[TB_YAW_Z];  //orientation about z-axis



//Gyrodometry --------------------------------------------------------------
float rect_gamma = mb_clamp_radians(gamma);
//float rect_gamma = gamma;
float orient_diff = gyro_heading - (rect_gamma + gamma_offset); // difference in odometry and gyro heading determinations
float threshold_diff = 2*M_PI/180; //threshold for difference in odometry and gyro heading determinations
float oneSeventy = (float)(170*M_PI/180);
float heading;

if(fabs(orient_diff) > threshold_diff && fabs(gyro_heading) < oneSeventy){
    heading = gyro_heading;
    gamma_offset += orient_diff;
}
else{
     heading = mb_clamp_radians(rect_gamma + gamma_offset);
     //heading = rect_gamma + gamma_offset;
}

//printf("gamma: %f\n", rect_gamma);

//update states
mb_state->left_encoder = Lw_enc;
mb_state->right_encoder = Rw_enc;
mb_state->theta = theta;
mb_state->phi = phi;
mb_state->gamma = heading; //with gyrodometry
//mb_state->gamma = rect_gamma; //without gyrodometry

//calculate X, Y position using odometry ------------------------------------

//change in wheel angle travel
float d_Lw_phi = 2*M_PI*(Lw_enc - prev_Lw_enc) / (ENCODER_RES*GEAR_RATIO*ENC_1_POL); // change in wheel angle left wheel (rad)
float d_Rw_phi = 2*M_PI*(Rw_enc - prev_Rw_enc) / (ENCODER_RES*GEAR_RATIO*ENC_2_POL); // change in wheel angle right wheel (rad)

float encoder_error_remove(float enc_phi){
	if(fabs(enc_phi)<0.008)
		enc_phi=0;
	return enc_phi;
}

d_Lw_phi=encoder_error_remove(d_Lw_phi);
d_Rw_phi=encoder_error_remove(d_Rw_phi);


//distance wheels traveled
float d_Lw_dist = wheel_radius*d_Lw_phi; // distance the left wheel traveled
float d_Rw_dist = wheel_radius*d_Rw_phi; // distance the right wheel traveled


//distance body traveled
float d_gamma = (d_Lw_dist - d_Rw_dist) / WHEEL_BASE; //change in robot orientation (rad) <=> angle of curvature of travel (alpha)
float arc_dist = (d_Rw_dist + d_Lw_dist) /2; // arc distance the robot traveled (m)
float straight_dist = arc_dist; //(small angle assumption), approx. strait line dist from start to end robot position (m)
//theroetically exact straight line dist (no small angle assumption)
//float radius_of_curvature = arc_dist / d_gamma; 
//float straight_dist_true = 2.0*radius_of_curvature*sin(d_gamma/2);

//change in x,y pos
float d_x = straight_dist*cos(prev_gamma + d_gamma/2.0); // change in x position of the robot
float d_y = straight_dist*sin(prev_gamma + d_gamma/2.0); // change in y position of the robot

//(motion model) Update state varpsi = heading;iables -----
float x_pos = prev_x + d_x;
float y_pos = prev_y + d_y;


//update odometry variables
mb_odometry->x = x_pos;
mb_odometry->y = y_pos;
mb_odometry->phi_odo = phi;
mb_odometry->gamma = heading;
mb_odometry->right_encoder = Rw_enc;
mb_odometry->left_encoder = Lw_enc;
//mb_odometry->gyro_gamma = gyro_heading;
mb_odometry->gyro_gamma = orient_diff;

}


float mb_clamp_radians(float angle){
	float out_angle=angle;
	float pi = (float)(M_PI);
	float twopi = (float)(2*M_PI);
	int div = 0;

	int sign = 1;
	if(out_angle < 0){
		sign = -1;
	}
	while(abs(out_angle)>twopi){
		out_angle = out_angle - twopi*sign;
	}

	if (angle<-pi){
		out_angle = out_angle + twopi;	
	}
	else if(angle>pi){
		out_angle = out_angle - twopi;	
	}

    return out_angle;
}

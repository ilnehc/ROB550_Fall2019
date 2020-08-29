#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

typedef enum drive_mode_t{
    NOVICE,
    ADVANCED
}drive_mode_t;

typedef enum m_intput_mode_t{
    DSM,
    STDIN
}m_intput_mode_t;

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    double   theta;             // body angle (rad)
    double   phi;               // average wheel angle (rad)
    double   gamma;             // Body turn angle (rad)
    double   left_encoder;      // left encoder counts since last reading
    double   right_encoder;     // right encoder counts since last reading


    //outputs
    double  left_cmd;  //left wheel command [-1..1]
    double  right_cmd; //right wheel command [-1..1]

    float opti_x;
    float opti_y;
    float opti_roll;
    float opti_pitch;
    float opti_yaw;

    int mb_saturated;

    //TODO: Add more variables to this state as needed
};

typedef struct mb_setpoints mb_setpoints_t;

struct mb_setpoints{
    double theta;		///< body lean angle (rad)
	double phi;		///< wheel position (rad)
	double phi_dot;		///< rate at which phi reference updates (rad/s)
	double gamma;		///< body turn angle (rad)
	double gamma_dot;	///< rate at which gamma setpoint updates (rad/s)
    drive_mode_t drive_mode_t;

    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int manual_ctl;
};

typedef struct mb_odometry mb_odometry_t;

struct mb_odometry{

    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float gamma;      //heading from initialization in rad
    float phi_odo;
    int left_encoder; //encoder value of left wheel
    int right_encoder; //encoder value of right wheel
    float gyro_gamma;  //gyro heading of previous value (if gyro isn't perfectly zero'd)
};

#endif

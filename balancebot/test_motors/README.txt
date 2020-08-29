test_motors.c : A basic script to drive the motors to check that the functions in ../common/mb_motor.c are function. It is also helps for checking that the motor defs in ../common/mb_defs.h are behaving as the user desires them to be sign wise. It is compiled in ../bin.

test_motors_params : Runs an extensive script for 1)A software check for the sanity of the encoders and motors and 2)Obtaining the motor parameters. This is a pre-compiled binary and can immediately be excuted within this folder. A copy of it is present in ../bin for the sake of having all compiled code in a single folder. There is no script provided as it remains an exercise for the report to describe how the motor parameters are obtained. The binary is called as:

sudo ./test_motors_param <RIGHT_MOTOR_RESISTANCE> <LEFT_MOTOR_RESISTANCE>

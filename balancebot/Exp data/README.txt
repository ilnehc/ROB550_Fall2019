
All compiled binaries are saved inside this folder for the user to run them. There is a pid and tf cfg files that the user may use to load their gains and transfer function info into their code. The advantage of using these files over hard-coded gain values is the lack of the necessity to complie code everytime a gain is tweaked, speeding up the process.

test_motors_params : Runs an extensive script for 1)A software check for the sanity of the encoders and motors and 2)Obtaining the motor parameters. This is a pre-compiled binary and can immediately be excuted within this folder. A copy of it is present in ../test_motors. There is no script provided as it remains an exercise for the report to describe how the motor parameters are obtained. The binary is called as:

sudo ./test_motors_param <RIGHT_MOTOR_RESISTANCE> <LEFT_MOTOR_RESISTANCE>

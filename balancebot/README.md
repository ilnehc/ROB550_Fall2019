## Balancebot

See our competition video: https://www.youtube.com/watch?v=eZcrrMhK1TI&feature=youtu.be

/******************************************************************************/

/*                           Balancebot Template Code                         */
                           
/*                           pgaskell@umich.edu                               */
       
*******************************************************************************/

=bin/			             : Binaries folder

=balanceebot/balanceebot.c/.h : Main setup and threads

=test_motors/test_motors.c/.h : Program to test motor implementation

=common/mb_controller.c/.h    : Contoller for manual and autonomous nav

=common/mb_defs.h             : Define hardware config

=common/mb_motors.c/.h        : Motor functions to be used by balancebot

=common/mb_odometry.c/.h	     : Odometry functions

=optitrack/		     : optitrack driver/server

=xbee_serial		     : xbee serial optitrack client code

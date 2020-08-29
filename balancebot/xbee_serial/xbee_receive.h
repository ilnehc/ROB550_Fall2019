#include "xbee_serial.h"
#include <stdio.h>
#include <unistd.h> // read / write
#include <stdlib.h>	//one of these two is for memcpy
#include <string.h>
#include <stdint.h>

// Below for PRId64
#include <inttypes.h>

#include "xbee_packet_t.h"

#define MSG_RATE_HZ 50

extern xbee_packet_t xbeeMsg;
extern int xbee_portID;

#define SERIALPORT "/dev/ttyO5"
#define BAUDRATE 57600

int XBEE_init(int baudrate);
int XBEE_getData();
void XBEE_printData();
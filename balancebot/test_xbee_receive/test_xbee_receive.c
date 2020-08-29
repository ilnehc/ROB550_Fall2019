#include "../xbee_serial/xbee_receive.h"

xbee_packet_t xbeeMsg;
int XBEE_portID;

int main(){	
  int baudRate = BAUDRATE;  
  // Begin Printing
  printf("\n");
  printf("   Time   |");
  printf("    X     |");
  printf("    Y     |");
  printf("    Z     |");
  printf("    qx    |");
  printf("    qy    |");
  printf("    qz    |");
  printf("    qw    |");
  printf(" valid |");
  printf("\n");
  
  XBEE_init(baudRate);
  while(1){
    XBEE_getData();
    XBEE_printData();
    usleep(1E6/MSG_RATE_HZ);
  }
  
  return 0;
}
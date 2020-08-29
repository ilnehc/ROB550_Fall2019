#include "xbee_packet_t.h"

void printXBeeMsg(xbee_packet_t &msg) {
  printf("\r");

  // Time
  if      (msg.time < 1000000)   printf("   %u |",msg.time);
  else if (msg.time < 10000000)  printf("  %u |",msg.time);
  else if (msg.time < 100000000) printf(" %u |",msg.time);
  else                         printf("%u |",msg.time);			
  
  // XYZ
  if( msg.x < 0)
    printf("%7.6f |",msg.x);
  else        
    printf(" %7.6f |",msg.x);
  if( msg.y < 0)
    printf("%7.6f |",msg.y);
  else
    printf(" %7.6f |",msg.y);
  if( msg.z < 0)
    printf("%7.6f |",msg.z);
  else
    printf(" %7.6f |",msg.z);
  
  // Quaternion
  if( msg.qx < 0)
    printf("%7.6f |",msg.qx);
  else
    printf(" %7.6f |",msg.qx);
  if( msg.qy < 0)
    printf("%7.6f |",msg.qy);
  else
    printf(" %7.6f |",msg.qy);
  if( msg.qz < 0)
    printf("%7.6f |",msg.qz);
  else
    printf(" %7.6f |",msg.qz);
  if( msg.qw < 0)
    printf("%7.6f |",msg.qw);
  else
    printf(" %7.6f |",msg.qw);
  
  // Tracking Valid
  printf("   %u   |",msg.trackingValid);   
  
  fflush(stdout);
}



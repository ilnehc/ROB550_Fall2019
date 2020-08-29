// Updated code (Feb 2019)
//
// Using data structure for packets with:
// Two start bytes:  0x81, 0xA1
// [Not included: Message ID (one byte), Message payload size (one byte) since we only have one message type]
// Message data (xbee_packet_t length)
// Fletcher-16 checksum (two bytes) computed starting with Message payload size
//
// Note:  This MBin protocol is commonly used on embedded serial devices subject to errors

#include "common/serial.h"
#include <iostream>
#include <unistd.h> // read / write

#include <cstdlib>	//one of these two is for memcpy
#include <cstring>
#include <stdint.h>
#include <string> //stoi

// Below for PRId64
#include <cinttypes>

#include "xbee_packet_t.h"

using std::cout;
using std::endl;

/**
* Data structure: A rigid body has a unique id, a 3D position, and a quaternion defining the orientation.
*/

int getData(xbee_packet_t &msg);
void printData(xbee_packet_t &msg);
void XBEE_readRingBuffer(xbee_packet_t &msg);

unsigned char c = (unsigned char) 'D';

// XBee Serial Port, Ring Buffer Variables
#define XBEE_RING_BUFSIZE 256
#define XBEE_RING_INC(a)(a<(XBEE_RING_BUFSIZE-1))?(a+1):0  // Increment ring buffer index macro
int XBEE_portID;
int XBEE_ring_overflow=0, XBEE_rdIndex=0, XBEE_wrIndex=0;
unsigned char XBEE_ringbuffer[XBEE_RING_BUFSIZE];

int main(int argc, char** argv) {	
  xbee_packet_t theMsg;
  int baudRate = 115200;  

  if(argc < 2) {
    cout << "Usage: " << endl;
    cout << " ./receiveSerial <Serial Port> " << endl << endl;
    cout << " ./receiveSerial <Serial Port> <Baudrate> " << endl << endl;
    cout << " <Serial Port> = /dev/ttyUSB0, etc..." << endl;
    cout << " <Baudrate> = 9600, 19200, etc ..." << endl;
    return 1;
  }
  if(argc == 3)
    baudRate = std::stoi(argv[2]);  

  XBEE_portID = serial_open(argv[1],baudRate,0); 	// blocking == 0 now,
  if(XBEE_portID == -1)    {
    cout << "Failed to open Serial Port" << endl;
    return 1;
  }
  
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
  
  while(1)
    getData(theMsg);
  
  return 0;
}


// Read and print message received from XBee; use ring buffer to assure no data loss
int getData(xbee_packet_t &msg)
{
  int k;
  unsigned char buffer;

  // Populate ring buffer
  for (k=0;k<XBEE_RING_BUFSIZE;k++) {
    if (XBEE_ring_overflow) { // Overflow condition on last read attempt
      if (XBEE_rdIndex == XBEE_wrIndex) { // Indices equal:  overflow still in effect
	return -1;  // Do not read data; reading data would cause overflow
	break;
      } else
	XBEE_ring_overflow = 0;  // Reset overflow flag
    }

    if (read(XBEE_portID, &buffer, 1) > 0) { // Read from buffer: returns 0 or -1 if no data
      XBEE_ringbuffer[XBEE_wrIndex] = buffer;
      XBEE_wrIndex = XBEE_RING_INC(XBEE_wrIndex);
      if (XBEE_wrIndex == XBEE_rdIndex) XBEE_ring_overflow = 1;
    } else
      break;
  }

  // Read and process data in ring buffer; print data
  XBEE_readRingBuffer(msg);
  return 0;
}

////////////////////////////////////
// XBEE_readRingBuffer()
#define XBEE_startByte1 0x81
#define XBEE_startByte2 0xA1
void XBEE_readRingBuffer(xbee_packet_t &msg)
{
  static unsigned char msgState = 0, msglength = 0;
  static unsigned char msgdata[_OPTI_DATA_LENGTH], ck0, ck1;

  while(XBEE_ring_overflow || (XBEE_rdIndex != XBEE_wrIndex)) { //Don't get ahead of the receiving data 
    if (XBEE_ring_overflow) XBEE_ring_overflow = 0; // Reset buffer overflow flag

    // Case 0:  Current character is first message header byte
    if((XBEE_ringbuffer[XBEE_rdIndex] == XBEE_startByte1) && !msgState) {
      msgState = 1; 
      msglength = 0;
    }
     
    // Case 1:  Current character is second message header byte
    else if (msgState == 1) {
      if (XBEE_ringbuffer[XBEE_rdIndex] == XBEE_startByte2) msgState = 2; 
      else msgState = 0;  // Bad message
      ck0 = 0;  // Initialize Fletcher checksum bytes
      ck1 = 0;
      msglength = 0;  // Initialize number of message payload data bytes read thusfar
    }
       
    // Case 2:  Read data bytes into msgdata[] array
    else if (msgState == 2) {
      msgdata[msglength++] = XBEE_ringbuffer[XBEE_rdIndex];
      ck0 += XBEE_ringbuffer[XBEE_rdIndex];
      ck1 += ck0;
      if (msglength == _OPTI_DATA_LENGTH) msgState = 3; // Done reading data
    }
    
    // Case 3:  Read & check the first checksum byte
    // Throw away data if full checksum doesn't match
    else if (msgState == 3) {
      if (ck0 != XBEE_ringbuffer[XBEE_rdIndex]) msgState = 0;
      else msgState = 4;
    }

    // Case 4:  Read & check the second checksum byte
    else if (msgState == 4) {
      msgState = 0;  // Done reading message data
      if (ck1 == XBEE_ringbuffer[XBEE_rdIndex]) { // Valid message -- copy and print
	memcpy(&msg, msgdata, _OPTI_DATA_LENGTH); 
	printData(msg);
      }
    }
    XBEE_rdIndex = XBEE_RING_INC(XBEE_rdIndex);
  }  

  return;
}

///////////////////////////////////////
void printData(xbee_packet_t &msg)
{
  // Print to terminal
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
  printf("   %d   |",msg.trackingValid);
    
  fflush(stdout);
}	






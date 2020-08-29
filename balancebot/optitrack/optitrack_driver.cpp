// Updated code (Feb 2019):  Use more robust serial message structure with Fletcher-16
#include <optitrack/optitrack.hpp>
#include <optitrack/common/getopt.h>
#include <optitrack/common/timestamp.h>
#include <optitrack/common/serial.h>

#include <errno.h>	//Errors for read/write
#include <unistd.h> // read / write / sleep
#include <stdio.h>

#include <cstdlib>
#include <cstring>

#include <netinet/in.h>
#include <sys/socket.h>
#include <iostream>
#include <cmath>

#include <cinttypes>
#include <inttypes.h>
#include "Quaternion.h"

#include "xbee_packet.h"
void initLogging(FILE **fpinput);

int main(int argc, char** argv)
{
  const char* kInterfaceArg = "interface";
  const char* kSerialBaudArg = "baudrate";
  const char* kVerbose = "verbose";
  const char* kLogging = "logging";    
  const char* kTransform = "transform";
  const char* kSerialPort = "serialPort";
  const char* kTestingFakeData  = "TestingFakeData";
  const char* kXbeeAddrArg = "xbeeAddr";
  const char* kRigidBodyArg = "RigidBodyID";
  
  getopt_t* gopt = getopt_create();
  getopt_add_bool(gopt, 'h', "help", 0, "Display this help message.\n");
  getopt_add_string(gopt, 'i', kInterfaceArg, "192.168.1.110", "Local network interface for connecting to Optitrack network"); 
  getopt_add_int(gopt, 'b', kSerialBaudArg, "57600", "Serial baudrate for communication via XBee");
  getopt_add_bool(gopt,'v',kVerbose, 0, "Print to terminal the Optitrack data");
  getopt_add_bool(gopt,'l',kLogging, 0, "Save data to logfile");
  getopt_add_bool(gopt,'t',kTransform, 0, "Transform data from Y-Up to NED Frame");
  getopt_add_string(gopt, 's',kSerialPort, "/dev/ttyUSB0", "Serial port used to send the XBee packets out");  
  getopt_add_bool(gopt, 'T',kTestingFakeData, 0, "Send fake, hardcoded data instead of optitrack for testing");  
  getopt_add_int(gopt,'x',kXbeeAddrArg, "1", "Address of target XBee");
  getopt_add_int(gopt,'r',kRigidBodyArg, "1", "ID of rigid body to publish.");
  
  if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
    printf("Usage: %s [options]", argv[0]);
    getopt_do_usage(gopt);
    return 1;
  }

  std::string interface = getopt_get_string(gopt, kInterfaceArg);
  int baudRate = getopt_get_int(gopt, kSerialBaudArg);
  bool verbose = getopt_get_bool(gopt, kVerbose);
  bool logging = getopt_get_bool(gopt, kLogging);
  bool transform = getopt_get_bool(gopt, kTransform);
  std::string serialPort = getopt_get_string(gopt, kSerialPort);
  bool testingFakeData = getopt_get_bool(gopt, kTestingFakeData);
  int xbeeAddr = getopt_get_int(gopt, kXbeeAddrArg);
  int rigidBodyId = getopt_get_int(gopt, kRigidBodyArg);
  
  ////////////////////
  // XBee Serial Port Variables
  int XBEE_portID;  
  xbee_packet_t xb_msg;
  char dataPacket[_OPTI_PACKET_LENGTH];
  dataPacket[0] = 0x81;  dataPacket[1] = 0xA1;  // Two start bytes
  unsigned char ck0=0, ck1=0;
  
  // Open a serial port
  printf("serial = %s\n", serialPort.c_str());
  XBEE_portID = serial_open(serialPort.c_str(),baudRate,1); 	// blocking while sending
  if(XBEE_portID == -1)  {
    printf("Failed to open Serial Port");
    return 1;
  }
  
  // Configure XBee Destination Address
  printf("Programming XBee...\n");
  char config_mode[] = "+++";
  char dest_addr_read[] = "ATDL\r\n";
  char dest_addr_cmd[80];
  sprintf(dest_addr_cmd, "ATDL %d\r\n", xbeeAddr);
  char cmd_null[] = "ATCN\r\n";
  char xbee_resp[] = "OK";
  char resp[10];
  write(XBEE_portID,config_mode,sizeof(config_mode)-1);
  usleep(1E5);
  read(XBEE_portID,resp,sizeof(resp));
  if(!strcmp(resp,xbee_resp))
    printf("Received Incorrect Response\n");
  write(XBEE_portID,dest_addr_cmd,sizeof(dest_addr_cmd)-1);
  usleep(1E5);
  read(XBEE_portID,resp,sizeof(resp));
  if(!strcmp(resp,xbee_resp))
    printf("Received Incorrect Response\n");
  write(XBEE_portID,dest_addr_read,sizeof(dest_addr_read)-1);
  usleep(1E5);
  read(XBEE_portID,resp,sizeof(resp));
  char dest_addr[10];
  sprintf(dest_addr,"%d",xbeeAddr);
  if(!strcmp(resp,dest_addr))
    printf("Received Incorrect Response\n");  
  write(XBEE_portID,cmd_null,sizeof(cmd_null)-1);
  usleep(1E5);
  read(XBEE_portID,resp,sizeof(resp));
  if(!strcmp(resp,xbee_resp))
    printf("Recieved Incorrect Response\n");
  printf("RESP:%s\n",resp);
  
  // Quat Transformation
  Quat Q_rotx_90;   // Rot about x by 90 degrees
  Q_rotx_90.w = 0.707107;
  Q_rotx_90.x = -0.707107;
  Q_rotx_90.y = 0;
  Q_rotx_90.z = 0; 

  
  Quat Q_rotx_90_inv;
  Q_rotx_90_inv = quatInv(Q_rotx_90);
  
  printf("Data size = %d\n",(int) sizeof(xbee_packet_t));
  if(verbose)	{
    // Printf Headers
    printf("\n");
    //printf("           |             Position              |                 RPY               |                  Quaternion                   |\n");      
    printf("   Time   |");
    printf("     x    |");
    printf("     y    |");
    printf("     z    |");
    printf("    qx    |");
    printf("    qy    |");
    printf("    qz    |");
    printf("    qw    |");
    
    printf("\n");
  }
  	
  // Writing to Logfile
  FILE *fpblah; 
  if(logging) 
    initLogging(&fpblah);

  // Grab Initial Time
  int64_t init_time64_u = utime_now();
  int64_t time64_u = utime_now();
  uint32_t time_u = (uint32_t) (time64_u - init_time64_u);
  
  
  //                      Testing Code (Fake Data)                           
  if (testingFakeData) {
    // Send data at "60 Hz" = 16.67 ms =  16,670 us
    // Send data at "10 Hz" = 100 ms = 100,000 us
    unsigned int microseconds = 16670;
    float testZero = 0;
    
    while (1) {
      usleep(microseconds);              
      time64_u = utime_now();
      time_u = (uint32_t) (time64_u - init_time64_u);
      
      // Construct XBee Packet
      xb_msg.time = time_u;
      xb_msg.x = testZero;
      xb_msg.y = testZero;
      xb_msg.z = testZero;
      xb_msg.qx = testZero;
      xb_msg.qy = testZero;
      xb_msg.qz = testZero;
      xb_msg.qw = testZero;
      xb_msg.trackingValid = 1;

      // Construct Serial Message
      // char* ptr = dataPacket;
      memcpy(dataPacket+2, &xb_msg, _OPTI_DATA_LENGTH);
      ck0=0; ck1=0;  // Fletcher-16 checksum
      for (int i=0; i < (int)  _OPTI_DATA_LENGTH; i++) {
	ck0 += dataPacket[i+2];
	ck1 += ck0;
      }
      dataPacket[_OPTI_DATA_LENGTH + 2] = ck0;
      dataPacket[_OPTI_DATA_LENGTH + 3] = ck1;
      
      // send serial message	(returns # of bytes sent on success)
      if(write(XBEE_portID,dataPacket,_OPTI_PACKET_LENGTH) > 0) {
	if(verbose)	    printXBeeMsg(xb_msg);  
	// "flush" the data 
	fsync(XBEE_portID);
      }			else			
	printf("Error: %d \n",errno);			
    }
  }

  // If there's no interface specified, then we'll need to guess
    if (interface.length() == 0)
      interface = guess_optitrack_network_interface();
    // If there's still no interface, we have a problem
    if (interface.length() == 0) {
      printf("[optitrack_driver] error could not determine network interface for receiving multicast packets.\n");
      return -1;
    }
    
    SOCKET dataSocket = create_optitrack_data_socket(interface, PORT_DATA);    
    if (dataSocket == -1) {
      printf("[optitrack_driver] error failed to create socket for interface %s:%d\n", interface.c_str(), PORT_DATA);
      return -1;
    } else {
      printf("[optitrack_driver] successfully created socket for interface %s:%d\n", interface.c_str(), PORT_DATA);
    }
    
    // Code from DataListenThread function in PacketClient.cpp
    char packet[20000];
    socklen_t addrLen = sizeof(sockaddr);
    sockaddr_in incomingAddress;
    std::vector<optitrack_message_t> incomingMessages;
        
    while (1) {
      // Block until we receive a datagram from the network
      recvfrom(dataSocket, packet, sizeof(packet), 0, (sockaddr*)&incomingAddress, &addrLen);
      incomingMessages = parse_optitrack_packet_into_messages(packet, sizeof(packet));
      for(auto& msg : incomingMessages) {
        if(msg.id == rigidBodyId){
          time64_u = utime_now();
          time_u = (uint32_t) (time64_u - init_time64_u);
          
          // Transform the data from Optitrack "Y-UP" To "North East Down" if selected
          if (transform) 
            frameTransformation(msg, Q_rotx_90, Q_rotx_90_inv);

          // Construct XBee Packet
          xb_msg.time = time_u;
          xb_msg.x = msg.x;
          xb_msg.y = msg.y;
          xb_msg.z = msg.z;
          xb_msg.qx = msg.qx;
          xb_msg.qy = msg.qy;
          xb_msg.qz = msg.qz;
          xb_msg.qw = msg.qw;
          xb_msg.trackingValid = (uint32_t) msg.trackingValid;

          // Construct Serial Message
          // char* ptr = dataPacket;
          memcpy(dataPacket+2, &xb_msg, _OPTI_DATA_LENGTH);
          ck0=0; ck1=0;  // Fletcher-16 checksum
          for (int i=0; i < (int) _OPTI_DATA_LENGTH; i++) {
            ck0 += dataPacket[i+2];
            ck1 += ck0;
          }
          dataPacket[_OPTI_DATA_LENGTH + 2] = ck0;
          dataPacket[_OPTI_DATA_LENGTH + 3] = ck1;
          
          // send serial message	(returns # of bytes sent on success)
          if(write(XBEE_portID,dataPacket,_OPTI_PACKET_LENGTH) > 0) {
            if(verbose)	    printXBeeMsg(xb_msg);  
            // "flush" the data 
            fsync(XBEE_portID);
          }			else			
            printf("Error: %d \n",errno);			
          
          // Write to file
          if(logging)		{
            fprintf(fpblah,"%u, ",time_u);			
            fprintf(fpblah,"%7.6f, ",msg.x);
            fprintf(fpblah,"%7.6f, ",msg.y);
            fprintf(fpblah,"%7.6f, ",msg.z);
            fprintf(fpblah,"%7.6f, ",msg.qx);
            fprintf(fpblah,"%7.6f, ",msg.qy);
            fprintf(fpblah,"%7.6f, ",msg.qz);
            fprintf(fpblah,"%7.6f, ",msg.qw);
            fprintf(fpblah,"%u,  ",xb_msg.trackingValid);
            fprintf(fpblah,"\n");
          }
        }
      }
    }
    
    // Cleanup options now that we've parsed everything we need
    getopt_destroy(gopt);
    fclose(fpblah);
    return 0;
}


// Initialize data log file
void initLogging(FILE **fpinput) {
    *fpinput = fopen("optitrack_logfile.csv", "w");
    FILE *fp = *fpinput;
    printf("file open.\n");
    // Data
    fprintf(fp, "u_Time, "); 
    fprintf(fp, "x, ");
    fprintf(fp, "y, ");
    fprintf(fp, "z, ");
    fprintf(fp, "qx, ");
    fprintf(fp, "qy, ");
    fprintf(fp, "qz, ");
    fprintf(fp, "qw, ");
    
    // End of Line
    fprintf(fp,"\n");
    fflush(fp);
}


#include <stdint.h>
// XBee Packet
typedef struct xbee_packet_t
{
    uint32_t time;   ///< Unique id for the rigid body being described
    float x;        ///< x-position in the Optitrack frame
    float y;        ///< y-position in the Optitrack frame
    float z;        ///< z-position in the Optitrack frame
    float qx;       ///< qx of quaternion
    float qy;       ///< qy of quaternion
    float qz;       ///< qz of quaternion
    float qw;       ///< qw of quaternion
    uint32_t trackingValid;   // (bool) of whether or not tracking was valid (0 or 1)
} xbee_packet_t;

#define NUM_FRAMING_BYTES 4                 // 2 START bytes + 2 Fletcher-16 checksum bytes
#define OPTI_DATA_LENGTH           sizeof(struct xbee_packet_t)      // Actual Packet Being Sent
#define OPTI_PACKET_LENGTH	    _OPTI_DATA_LENGTH + NUM_FRAMING_BYTES
#include "optitrack.hpp"


struct Quat {
    float w;
    float x;
    float y;
    float z;
};

Quat quatInv(Quat Q1) {
    Q1.x = -1 * Q1.x;
    Q1.y = -1 * Q1.y;
    Q1.z = -1 * Q1.z;    

    return Q1;
};

Quat quatMultiply(Quat Q1, Quat Q2) {
    Quat Q3;

    // w component
    Q3.w = Q1.w*Q2.w - Q1.x*Q2.x - Q1.y*Q2.y - Q1.z*Q2.z;

    // x component
    Q3.x = Q1.w*Q2.x + Q1.x*Q2.w + Q1.y*Q2.z - Q1.z*Q2.y;

    // y component
    Q3.y = Q1.w*Q2.y + Q1.y*Q2.w + Q1.z*Q2.x - Q1.x*Q2.z;

    // z component
    Q3.z = Q1.w*Q2.z + Q1.z*Q2.w + Q1.x*Q2.y - Q1.y*Q2.x;
 
    return Q3;   
};

Quat quatMultiply3(Quat Q1, Quat Q2, Quat Q3) {
    Quat Q4;

    Q4 = quatMultiply( quatMultiply(Q1,Q2), Q3);

    return Q4;
};


void frameTransformation(optitrack_message_t &msg, Quat Q_Quad_Opti, Quat Q_Quad_Opti_inv) {
    Quat Q1_Opti, P1_Opti, Q1_Quad, P1_Quad;

    // Construct Quat Object from msg for actual quaternion
    Q1_Opti.w = msg.qw;
    Q1_Opti.x = msg.qx;
    Q1_Opti.y = msg.qy;
    Q1_Opti.z = msg.qz;

    // Construct Quat Object from msg for position
    P1_Opti.w = 0;
    P1_Opti.x = msg.x;
    P1_Opti.y = msg.y;
    P1_Opti.z = msg.z;

    // Transform Data from Opti Frame to Quad frame (assume no position offset)
    Q1_Quad = quatMultiply3(Q_Quad_Opti, Q1_Opti, Q_Quad_Opti_inv);
    P1_Quad = quatMultiply3(Q_Quad_Opti, P1_Opti, Q_Quad_Opti_inv);
   
    // Overwrite the msg with the transformed data
    msg.qw = Q1_Quad.w;
    msg.qx = Q1_Quad.x;
    msg.qy = Q1_Quad.y;
    msg.qz = Q1_Quad.z;
    msg.x  = P1_Quad.x;
    msg.y  = P1_Quad.y;
    msg.z  = P1_Quad.z;
}

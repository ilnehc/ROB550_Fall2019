// file: read_log.cpp
//
// LCM example program.  Demonstrates how to read and decode messages directly
// from a log file in C++.  It is also possible to use the log file provider --
// see the documentation on the LCM class for details on that method.
//
// compile with:
//  $ g++ -o read_log read_log.cpp -llcm
//
// On a system with pkg-config, you can also use:
//  $ g++ -o read_log read_log.cpp `pkg-config --cflags --libs lcm`

#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include "common/lcm_config.h"
//#include "exlcm/example_t.hpp"
#include "lcmtypes/pose_xyt_t.hpp"
#include "lcmtypes/lidar_t.hpp"
#include "lcmtypes/robot_path_t.hpp"
#include "lcmtypes/particle_t.hpp"
#include "lcmtypes/particles_t.hpp"

using namespace std;

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: read_log <logfile>\n");
        return 1;
    }

    // Open the log file.
    lcm::LogFile log(argv[1], "r");
    if (!log.good()) {
        perror("LogFile");
        fprintf(stderr, "couldn't open log file %s\n", argv[1]);
        return 1;
    }

    while (1) {
        // Read a log event.
        const lcm::LogEvent *event = log.readNextEvent();
        if (!event)
            break;

        // Only process messages on the EXAMPLE channel.
        if (event->channel != "SLAM_PARTICLES")
            continue;

        // Try to decode the message.
        particles_t msg;
        if (msg.decode(event->data, 0, event->datalen) != event->datalen)
            continue;

        printf("i    |      x       |       y     \n");
        for (int i=0; i<msg.num_particles; i++)
        {
            // Decode success!  Print out the message contents.
            //printf("Message:\n");
            printf("%d     |     %f     |%f    \n", i, msg.particles[i].pose.x, msg.particles[i].pose.y);
            // printf("  orientation = (%f, %f, %f, %f)\n", msg.orientation[0], msg.orientation[1],
            //        msg.orientation[2], msg.orientation[3]);
            // printf("  ranges:");
            // for (int i = 0; i < msg.num_ranges; i++)
            //     printf(" %d", msg.ranges[i]);
            // printf("\n");
            // printf("  name        = '%s'\n", msg.name.c_str());
            // printf("  enabled     = %d\n", msg.enabled);
        }
        
    }

    // Log file is closed automatically when the log variable goes out of
    // scope.

    printf("done\n");
    return 0;
}

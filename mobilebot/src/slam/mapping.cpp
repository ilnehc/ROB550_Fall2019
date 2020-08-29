#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <cstdlib>
#include <math.h>       /* cos */

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{

    //printf("%f\n",map.logOdds(1,1));
    //map.setLogOdds(1,1,1)

    /*int32_t num_ranges;
    float   ranges[num_ranges];         // [m]
    float   thetas[num_ranges];         // [rad]
    int64_t times[num_ranges];          // [usec]
    float   intensities[num_ranges];    // no units
    */
    //float x = scan.thetas[1];
    //printf("%f\n",x);

    //const float grid_size = map.metersPerCell(); //.05; // 5cm
    float Kp = 10; // was 1;
    float Kn = 3.5;

    // float x_origin = map.originInGlobalFrame().x;
    // float y_origin = map.originInGlobalFrame().y;

    float x0_m = pose.x; // in meters
    float y0_m = pose.y;
    float body_theta = pose.theta;

    /*
    int x0 = global_position_to_grid_cell(x0_m, grid_size); // - x_origin; // - map.global_o....; // in grid coordinates
    int y0 = to_grid(y0_m, grid_size); // - y_origin;
    */

    Point<double> pose0 = Point<double>(x0_m, y0_m);
    Point<int> p0 = global_position_to_grid_cell(pose0, map);
    int x0 = p0.x;
    int y0 = p0.y;

    // for each laser beam, use breshentham to update map
    int num_lasers = scan.num_ranges;
    for (int i=1; i<num_lasers; i++){
        float r = scan.ranges[i];
        float laser_theta = scan.thetas[i];
        float x1_m = x0_m + r * cos(laser_theta - body_theta); // subtracted out body angle 
        float y1_m = y0_m + r * sin(laser_theta - body_theta + 3.14159);

        Point<double> pose1 = Point<double>(x1_m,y1_m);
        Point<int> p1 = global_position_to_grid_cell(pose1, map);
        int x1 = p1.x;
        int y1 = p1.y;


        /*
        int x1 = to_grid(x1_m, grid_size); // - x_origin;
        int y1 = to_grid(y1_m, grid_size); // - y_origin;
        */



        float dx = std::abs(x1-x0);
        float dy = std::abs(y1-y0);
        int sx = x0<x1 ? 1 : -1;
        int sy = y0<y1 ? 1 : -1;
        float err = dx-dy;

        // x,y starting points
        int x = x0; // was float for some reason
        int y = y0;
        // change x and y until hit one of end points
        while(x != x1 || y != y1){
            if (map.isCellInGrid(x,y)) {
                //Update Odds at (x,y);
                float prev_odds = map.logOdds(x,y);
                // Clamp
                if (prev_odds - Kn < -127) {
                    map.setLogOdds(x,y,-127);
                } else {
                    map.setLogOdds(x,y,prev_odds - Kn);
                }
            }
            float e2 = 2*err;
            if (e2 >= -dy){
                err -= dy;
                x += sx;
            }
            if (e2 <= dx){
                err += dx;
                y += sy;
            }
        }
        // Last one, we want to add
        float prev_odds = map.logOdds(x1,y1);
        if (prev_odds + Kp > 127) {
            map.setLogOdds(x1,y1,127);
        } else {
            map.setLogOdds(x1,y1,prev_odds + Kp);
        }
    }
}

/*
int Mapping::to_grid(float x, float metersPerCell){ // float grid_size) {
    //return floor(x / grid_size);
    return x * metersPerCell;
}
*/


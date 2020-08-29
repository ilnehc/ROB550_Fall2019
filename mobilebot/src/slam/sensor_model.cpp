#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    int stride = 6; // can change this to optimize speed
    double odds = 0;

    float x0_m = sample.pose.x; // in meters
    float y0_m = sample.pose.y;
    float body_theta = sample.pose.theta;

    /*
    Point<double> pose0 = Point<double>(x0_m, y0_m);
    Point<int> p0 = global_position_to_grid_cell(pose0, map);
    int x0 = p0.x;
    int y0 = p0.y;
    */

   

    int num_lasers = scan.num_ranges;
    for (int i=1; i<num_lasers; i+=stride ){
        float r = scan.ranges[i];
        //if (r = scan.MaxRange)
        float laser_theta = scan.thetas[i];
        float x1_m = x0_m + r * cos(laser_theta - body_theta); // subtracted out body angle 
        float y1_m = y0_m + r * sin(laser_theta - body_theta + 3.14159);

        Point<double> pose1 = Point<double>(x1_m,y1_m);
        Point<int> p1 = global_position_to_grid_cell(pose1, map);
        int x1 = p1.x;
        int y1 = p1.y;

        // Check if final grid position is obstacle
        if (map.logOdds(x1,y1) > 70) {
            odds += map.logOdds(x1,y1);// + 127;
        } 
        
        //
        //else {
            // worry about nearby hits
            double nearby_odds = 0;
            for (int i=-1;i<=1;i++) {
                for (int j=-1;j<=1;j++) {
                    if (map.logOdds(x1+i,y1+j) > 70) {
                        nearby_odds += map.logOdds(x1+i,y1+j); // + 127;
                    }
                }
            }
            nearby_odds *= (1.0/8/2); // was 2
            odds += nearby_odds; // shift all values by 127
        //}
        //

    }

    return odds;
}

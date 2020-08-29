#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>
#include <vector>
using namespace std;

ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    vector<vector<int>> obs;
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    for (int i = 0; i < map.widthInCells(); i++)
        for (int j = 0; j < map.heightInCells(); j++)
        {
            if (map.logOdds(i,j) > 0)
            {
                distance(i, j) = 0;
                obs.push_back({i, j});
            }
        }
    for (int i = 0; i < map.widthInCells(); i++)
        for (int j = 0; j < map.heightInCells(); j++)
        {
            int dist = 2000;
            if (map.logOdds(i, j) < 0)
            {
                for (int k = 0; k < obs.size(); k++)
                {
                    if (dist > fabs(obs[k][0] - i) + fabs(obs[k][1] - j) + (1.414 - 2) * min(fabs(obs[k][0] - i), fabs(obs[k][1] - j)))
                    {
                        dist = fabs(obs[k][0] - i) + fabs(obs[k][1] - j) + (1.414 - 2) * min(fabs(obs[k][0] - i), fabs(obs[k][1] - j));

                        dist = 1.5 * dist;
                        //if(i == 2 && j == 2)
                        //printf("%d,%d,%d,%d\n",map.logOdds(i,j),obs[k][0],obs[k][1],dist);
                    }
                }
            distance(i, j) = dist * map.metersPerCell();
            }   
        }
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}

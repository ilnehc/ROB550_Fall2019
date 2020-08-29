#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <math.h>

class ObstacleDistanceGrid;

/**
* SearchParams defines the parameters to use when searching for a path. See associated comments for details
*/
struct SearchParams
{
    double minDistanceToObstacle;   ///< The minimum distance a robot can be from an obstacle before
                                    ///< a collision occurs
                                    
    double maxDistanceWithCost;     ///< The maximum distance from an obstacle that has an associated cost. The planned
                                    ///< path will attempt to stay at least this distance from obstacles unless it must
                                    ///< travel closer to actually find a path
                                    
    double distanceCostExponent;    ///< The exponent to apply to the distance cost, whose function is:
                                    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
};

struct Node
{
    int y;
    int x;
    int parentX;
    int parentY;
    float gCost;
    float hCost;
    float oCost; 
    float fCost;
};

inline bool operator < (const Node& lhs, const Node& rhs);
static bool isValid(int x, int y, const ObstacleDistanceGrid& distances,  const double minDist);
static bool isDestination(int x, int y, Node dest);
static double calculateH(int x, int y, Node dest);
static double calculateO(int x, int y, const ObstacleDistanceGrid& distances, const SearchParams& params);
bool isCellInGrid(int x, int y, const ObstacleDistanceGrid& distances);
/*
static robot_path_t makePath(array<array < Node, (distances.heightInCells())>, distances.widthInCells()> map, Node dest, const ObstacleDistanceGrid& distances)
{
    cout << "Found a Path" << endl;
    int x = dest.x;
    int y = dest.y;
    stack<pose_xyt_t> path;
    robot_path_t usablePath;

    while (!(map[x][y].parentX==x && map[x][y]==y) && map[x][y].x != -1 && map[x][y] != -1)
    {
        pose_xyt_t node;
        node.x = (map[x][y].x - distances.originInGlobalFrame().x) * distances.meterPerCell();
        node.y = (map[x][y].y - distances.originInGlobalFrame().y) * distances.meterPerCell();
        path.push(node);
        int tempX = map[x][y].parentX;
        int tempY = map[x][y].parentY;
        x = tempX;
        y = tempY;
    }
    pose_xyt_t node;
    node.x = (map[x][y].x - distances.originInGlobalFrame().x) * distances.meterPerCell();
    node.y = (map[x][y].y - distances.originInGlobalFrame().y) * distances.meterPerCell();
    path.push(node);

    while (!path.empty())
    {
        pose_xyt_t top = path.top();
        path.pop();
        usablePath.path.emplace_back(top);
    }
    return usablePath;
}
*/

/**
* search_for_path uses an A* search to find a path from the start to goal poses. The search assumes a circular robot
* 
* \param    start           Starting pose of the robot
* \param    goal            Desired goal pose of the robot
* \param    distances       Distance to the nearest obstacle for each cell in the grid
* \param    params          Parameters specifying the behavior of the A* search
* \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with just the initial
*   pose is returned, per the robot_path_t specification.
*/
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);

#endif // PLANNING_ASTAR_HPP

#include <planning/motion_planner.hpp>
#include <planning/astar.hpp>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <lcmtypes/robot_path_t.hpp>
#include <cmath>


MotionPlanner::MotionPlanner(const MotionPlannerParams& params)
: params_(params)
{
    setParams(params);
}


MotionPlanner::MotionPlanner(const MotionPlannerParams& params, const SearchParams& searchParams)
: params_(params)
, searchParams_(searchParams)
{
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start, 
                                     const pose_xyt_t& goal, 
                                     const SearchParams& searchParams) const
{
    // If the goal isn't valid, then no path can actually exist
    if(!isValidGoal(goal))
    {
        robot_path_t failedPath;
        failedPath.utime = utime_now();
        failedPath.path_length = 1;
        failedPath.path.push_back(start);

        std::cout << "INFO: path rejected due to invalid goal\n";        

        return failedPath;
    }
    
    // Otherwise, use A* to find the path
    return search_for_path(start, goal, distances_, searchParams);
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start, const pose_xyt_t& goal) const
{
    return planPath(start, goal, searchParams_);
}


bool MotionPlanner::isValidGoal(const pose_xyt_t& goal) const
{
    float dx = goal.x - prev_goal.x, dy = goal.y - prev_goal.y;
    float distanceFromPrev = std::sqrt(dx * dx + dy * dy);

    //if there's more than 1 frontier, don't go to a target that is within a robot diameter of the current pose
    //if(num_frontiers != 1 && distanceFromPrev < 2 * searchParams_.minDistanceToObstacle) return false;

    auto goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances_);

    // A valid goal is in the grid
    if(distances_.isCellInGrid(goalCell.x, goalCell.y))
    {
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan
        return distances_(goalCell.x, goalCell.y) > params_.robotRadius;
    }
    
    // A goal must be in the map for the robot to reach it
    return false;
}


bool MotionPlanner::isPathSafe(const robot_path_t& path) const
{

    ///////////// TODO: Implement your test for a safe path here //////////////////
    //if (path.path_length < 1) return false;
    if (path.path.empty()) return false;

    int stride = 1;

    //setPrevGoal(path.path[0]);

    // Just added this to make short paths impossible
    if (path.path_length < 8) { // was 4
        return false;
    }

    for (int i = 0; i < path.path.size(); i+=stride) {
        pose_xyt_t pose = path.path.at(i);

        // check if valid pose
        auto goalCell = global_position_to_grid_cell(Point<double>(pose.x, pose.y), distances_);
        // A valid goal is in the grid
        if(distances_.isCellInGrid(goalCell.x, goalCell.y))
        {
            // And is far enough from obstacles that the robot can physically occupy the space
            // Add an extra cell to account for discretization error and make motion a little safer by not trying to
            // completely snuggle up against the walls in the motion plan

            // closer to an obstacle than robot radius, so return false
            if (!distances_(goalCell.x, goalCell.y) > params_.robotRadius) return false;
        } else { 
            // not in the grid
            return false;
        }
        //
       //if (!isValidGoal(pose)) return false;
    }

    return true;
}


void MotionPlanner::setMap(const OccupancyGrid& map)
{
    distances_.setDistances(map);
}


void MotionPlanner::setParams(const MotionPlannerParams& params)
{
    searchParams_.minDistanceToObstacle = params_.robotRadius;
    searchParams_.maxDistanceWithCost = 10.0 * searchParams_.minDistanceToObstacle; // was 10, 8 seemed to make bigger
    searchParams_.distanceCostExponent = 1.0;
}

#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/grid_utils.hpp>
#include <array>
#include <stack>
#include <cfloat>
using namespace std;

robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{

    ////////////////// TODO: Implement your A* search here //////////////////////////
	vector< vector<Node>> allMap;
    Node startnode, dest;
    //printf("\n%f,%f\n%f,%f\n",start.x,start.y,goal.x,goal.y);
	//printf("%f,%f\n",distances.originInGlobalFrame().x,distances.originInGlobalFrame().y);
	robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    //convert to grid cell
    dest.x = static_cast<int>((goal.x - distances.originInGlobalFrame().x) * distances.cellsPerMeter());// + 1;
    dest.y = static_cast<int>((goal.y - distances.originInGlobalFrame().y) * distances.cellsPerMeter());// + 1;
    startnode.x = static_cast<int>((start.x - distances.originInGlobalFrame().x) * distances.cellsPerMeter());// + 1;
    startnode.y = static_cast<int>((start.y - distances.originInGlobalFrame().y) * distances.cellsPerMeter());// + 1;
    printf("\n%d,%d\n%d,%d\n",startnode.x,startnode.y,dest.x,dest.y);
	if (!isValid(dest.x, dest.y, distances, params.minDistanceToObstacle))
    {
		printf("Destination is cannot be reached\n");
		path.path_length = path.path.size();
		return path;
    }  
	
	if (!isValid(startnode.x, startnode.y, distances, params.minDistanceToObstacle))
    {
		printf("Origin is invalid\n");
		path.path_length = path.path.size();
		return path;
    } 
	     
	if(isDestination(startnode.x, startnode.y, dest))
    {
        printf("Already at Destination\n");
		path.path_length = path.path.size();
        return path;
    }
	//printf("\n1");
	
	if(!isCellInGrid(dest.x, dest.y, distances) || !isCellInGrid(startnode.x, startnode.y, distances))
	{
		printf("Start or Destination not in grid%f\n",distances.operator()(startnode.x,startnode.y));
		path.path_length = path.path.size();
        return path;
	}
	// else	
	 	//printf("\n2");
	// printf("\n3");
	// printf("\n4");
	bool closedList[distances.widthInCells()][distances.heightInCells()];
    
	//cout << "\n3";
    for (int x = 0; x < (distances.widthInCells()); x++) 
    {
		vector<Node> suptemp;
		for (int y = 0; y < (distances.heightInCells()); y++) {
			Node temp;
			temp.fCost = FLT_MAX;
			temp.gCost = FLT_MAX;
			temp.hCost = FLT_MAX;
			temp.parentX = -1;
			temp.parentY = -1;
			temp.x = x;
			temp.y = y;
			suptemp.push_back(temp);
			closedList[x][y] = false;
		}
		allMap.push_back(suptemp);
	}
	//cout << "\n4";
    int x = startnode.x;
	int y = startnode.y;
	allMap[x][y].fCost = 0.0;
	allMap[x][y].gCost = 0.0;
	allMap[x][y].hCost = 0.0;
	allMap[x][y].parentX = x;
	allMap[x][y].parentY = y;
    vector<Node> openList;	
	openList.emplace_back(allMap[x][y]);
	bool destinationFound = false;
	//cout << "\n5";

    while (!openList.empty()&&openList.size()<(distances.widthInCells())*(distances.heightInCells())) 
    {
			Node node;
			do 
            {
				//This do-while loop could be replaced with extracting the first
				//element from a set, but you'd have to make the openList a set.
				//To be completely honest, I don't remember the reason why I do
				//it with a vector, but for now it's still an option, although
				//not as good as a set performance wise.
				float temp_f = FLT_MAX;
                float temp_h = FLT_MAX;
				vector<Node>::iterator itNode;
				for (vector<Node>::iterator it = openList.begin();
					it != openList.end(); it = next(it)) 
                {
					Node n = *it;
					if (n.fCost < temp_f) 
                    {
						temp_f = n.fCost;
                        temp_h = n.hCost;
						itNode = it;
					}
                    else if (n.fCost == temp_f) 
                    {
                        if (n.hCost < temp_h) {
                            temp_f = n.fCost;
                            temp_h = n.hCost;
						    itNode = it;
                        }
					}
				}
				node = *itNode;
				openList.erase(itNode);
			} while (isValid(node.x, node.y, distances, params.minDistanceToObstacle) == false);

			x = node.x;
			y = node.y;
			closedList[x][y] = true;
			//printf("%f",params.minDistanceToObstacle);

			//For each neighbour starting from North-West to South-East
			for (int newX = -1; newX <= 1; newX++) 
            {
				for (int newY = -1; newY <= 1; newY++) 
                {
					double gNew, hNew, oNew, fNew;
					if (isValid(x + newX, y + newY, distances, params.minDistanceToObstacle)) //add isValid()
                    {
						if (isDestination(x + newX, y + newY, dest)) //add isGoal
						{
							//Destination found - make path
							allMap[x + newX][y + newY].parentX = x;
							allMap[x + newX][y + newY].parentY = y;
							destinationFound = true;
							//path = makePath(allMap, dest, distances);

							/////// make a path
							cout << "Found a Path" << endl;
							int x = dest.x;
							int y = dest.y;
							stack<pose_xyt_t> path;
							robot_path_t usablePath;
							usablePath.utime = start.utime;

							while (!(allMap[x][y].parentX==x && allMap[x][y].parentY==y) && allMap[x][y].x != -1 && allMap[x][y].y != -1)
							{
								pose_xyt_t tempnode;
								//printf("X: %d, Y: %d\n", x, y);
								tempnode.x = static_cast<float>((allMap[x][y].x) * distances.metersPerCell() + distances.originInGlobalFrame().x);
								tempnode.y = static_cast<float>((allMap[x][y].y) * distances.metersPerCell() + distances.originInGlobalFrame().y);
								path.push(tempnode);

								// std::cout << "F cost: " << allMap[x][y].fCost << "\n";
								// std::cout << "O cost: " << allMap[x][y].oCost << "\n";
								// std::cout << "H cost: " << allMap[x][y].hCost << "\n";


								int tempX = allMap[x][y].parentX;
								int tempY = allMap[x][y].parentY;
								x = tempX;
								y = tempY;
								if (!isValid(x, y, distances, params.minDistanceToObstacle))
								{
									cout << "Path found is invalid" << "\n";
									robot_path_t invalidpath;
									invalidpath.utime = start.utime;
    								invalidpath.path.push_back(start);   
									invalidpath.path_length = invalidpath.path.size();
									return invalidpath;
								}
							}
							pose_xyt_t node;
							node.x = static_cast<float>((allMap[x][y].x) * distances.metersPerCell() + distances.originInGlobalFrame().x);
							node.y = static_cast<float>((allMap[x][y].y) * distances.metersPerCell() + distances.originInGlobalFrame().y);
							path.push(node);

							while (!path.empty())
							{
								pose_xyt_t top = path.top();
								path.pop();
								usablePath.path.emplace_back(top);
							}
							usablePath.path_length = usablePath.path.size();

							// for(int i = 0 ; i<usablePath.path_length; i++)
							// 	printf("X: %f, Y: %f\n", usablePath.path[i].x, usablePath.path[i].y);
							return usablePath;
							/*
							// delete points on the same line
							robot_path_t finalPath;
							finalPath.path.emplace_back(usablePath.path[0]);
							bool dir_change = 0;
							for (int i = 1; i < usablePath.path_length; i++)
							{
								bool dir_x = (usablePath.path[i].x == usablePath.path[i-1].x);
								bool dir_y = (usablePath.path[i].y == usablePath.path[i-1].y);
								if (!dir_x && !dir_y)
								{
									if (dir_change == 0)
									{
										finalPath.path.emplace_back(usablePath.path[i-1]);
										//finalPath.path.emplace_back(usablePath.path[i-1]);
										finalPath.path.emplace_back(usablePath.path[i]);
										//finalPath.path.emplace_back(usablePath.path[i]);
									}
									else
										finalPath.path.emplace_back(usablePath.path[i]);
										//finalPath.path.emplace_back(usablePath.path[i]);
									dir_change = 1;
								}
								else
									dir_change = 0;
							}
							finalPath.path.emplace_back(usablePath.path[usablePath.path_length-1]);
							//finalPath.path.emplace_back(usablePath.path[usablePath.path_length-1]);
							finalPath.path_length = finalPath.path.size();
							
							for(int i = 0 ; i<finalPath.path_length; i++)
								printf("X: %f, Y: %f\n", finalPath.path[i].x, finalPath.path[i].y);
							return finalPath; //usablePath;
							*/
						}
						else if (closedList[x + newX][y + newY] == false)
						{
							if(abs(newX)+abs(newY)==2)
								gNew=node.gCost + 1.4;
							else if (abs(newX)+abs(newY)==0)
								gNew = node.gCost; 
							else
								gNew = node.gCost + 1.0;
							hNew = calculateH(x + newX, y + newY, dest); //add Caluclate H
                            oNew = calculateO(x + newX, y + newY, distances, params);  //add Caluculate O
							
							//std::cout << "O new: " << oNew << "\n";
							
							fNew = gNew + hNew + oNew; //add Obstacle Cost
							// Check if this path is better than the one already present
							if (allMap[x + newX][y + newY].fCost == FLT_MAX ||
								allMap[x + newX][y + newY].fCost > fNew)
							{
								// Update the details of this neighbour node
								allMap[x + newX][y + newY].fCost = fNew;
								allMap[x + newX][y + newY].gCost = gNew;
								allMap[x + newX][y + newY].hCost = hNew;
								allMap[x + newX][y + newY].parentX = x;
								allMap[x + newX][y + newY].parentY = y;
								openList.emplace_back(allMap[x + newX][y + newY]);
							}
							//printf("x: %d, y: %d\n", x+newX, y+newY);
						}
					}
				}
			}
		}
        
	if (destinationFound == false) 
	{
		cout << "Destination not found" << endl;
		path.path_length = path.path.size();
		return path;
	}

	//path.path_length = path.path.size();
	//return path;
}

inline bool operator < (const Node& lhs, const Node& rhs)
{//We need to overload "<" to put our struct into a set
    return lhs.fCost < rhs.fCost;
}

static bool isValid(int x, int y, const ObstacleDistanceGrid& distances, const double minDist) 
{ //If our Node is an obstacle it is not valid
	//printf("%d,%d: %f, %f\n", x,y,distances.operator()(x,y), minDist);
	if (distances.operator()(x,y) >  minDist*1.000001) 
	{
		//printf("%f\n", minDist * distances.cellsPerMeter());
		if (x < 0 || y < 0 || x >= (distances.widthInCells()) || y >= (distances.heightInCells())) {
            return false;
        }
        return true;
    } 
	//printf("%d,%d: %f\n", x,y,distances.operator()(x,y));
    return false;
}

static bool isDestination(int x, int y, Node dest) 
{
    if (x == dest.x && y == dest.y) {
        return true;
    }
    return false;
}

static double calculateH(int x, int y, Node dest) 
{
    double H = (sqrt((x - dest.x)*(x - dest.x)
        + (y - dest.y)*(y - dest.y)));
    return H;
}

static double calculateO(int x, int y, const ObstacleDistanceGrid& distances, const SearchParams& params) 
{
    double dist = distances(x,y);
    if (dist >= params.maxDistanceWithCost)    return 0;
    else
    {
        ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
        ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
		double cost = pow((params.maxDistanceWithCost - dist)*200, params.distanceCostExponent); //200 for exploration // 20 a little low
		//std::cout << "O cost: " << cost << "\n";
		return cost;

        //return    pow(params.maxDistanceWithCost - dist, params.distanceCostExponent + 3); // added +1 
    }
    return 0.0;
}

bool isCellInGrid(int x, int y, const ObstacleDistanceGrid& distances)
{ 
    bool xCoordIsValid = (x >= 0) && (x < distances.widthInCells());
    bool yCoordIsValid = (y >= 0) && (y < distances.heightInCells());
	//printf("x:%d, y:%d\n", xCoordIsValid, yCoordIsValid);
    return (xCoordIsValid && yCoordIsValid);
}
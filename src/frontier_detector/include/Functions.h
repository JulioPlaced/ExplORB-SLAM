/*  Original work: http://wiki.ros.org/rrt_exploration
    Citation:
    Umari, H., & Mukhopadhyay, S. (2017, September).
    Autonomous robotic exploration based on multiple rapidly-exploring randomized trees.
    In 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 1396-1402). IEEE.
*/

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

#include <vector>
#include <stdlib.h>
#include <time.h>
#include <math.h>

// rdm class, for gentaring random flot numbers
class rdm{
int i;
public:
rdm();
float randomize();
};

//Norm function prototype
float Norm( std::vector<float> , std::vector<float> );

//sign function prototype
float sign(float );

//Nearest function prototype
std::vector<float> Nearest(  std::vector< std::vector<float>  > , std::vector<float> );

//Steer function prototype
std::vector<float> Steer(  std::vector<float>, std::vector<float>, float );

//gridValue function prototype
int gridValue(nav_msgs::OccupancyGrid &,std::vector<float>);

//ObstacleFree function prototype
char ObstacleFree(std::vector<float> , std::vector<float> & , nav_msgs::OccupancyGrid);

#endif //FUNCTIONS_H

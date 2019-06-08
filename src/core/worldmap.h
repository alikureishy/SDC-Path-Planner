#ifndef WORLD_MAP_H
#define WORLD_MAP_H

#include <iostream>
#include <string>
#include <vector>

using std::string;
using std::vector;

class WorldMap {
    public:
        WorldMap(const vector<double>& waypoints_x,
                 const vector<double>& waypoints_y, 
                 const vector<double>& waypoints_s, 
                 const vector<double>& waypoints_dx, 
                 const vector<double>& waypoints_dy,
                 double max_s) {

        }
};
typedef std::shared_ptr<WorldMap> WorldMap_;


#endif
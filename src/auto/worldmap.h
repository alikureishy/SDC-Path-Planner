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
             double max_s) :
                waypoints_x(waypoints_x),
                waypoints_y(waypoints_y),
                waypoints_s(waypoints_s),
                waypoints_dx(waypoints_dx),
                waypoints_dy(waypoints_dy)
             {
        this->max_s = max_s;
    }

    double getMaxS() const {
        return max_s;
    }

    const vector<double> &getWaypointsX() const {
        return waypoints_x;
    }

    const vector<double> &getWaypointsY() const {
        return waypoints_y;
    }

    const vector<double> &getWaypointsS() const {
        return waypoints_s;
    }

    const vector<double> &getWaypointsDx() const {
        return waypoints_dx;
    }

    const vector<double> &getWaypointsDy() const {
        return waypoints_dy;
    }

private:
    double max_s;
    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;
};


#endif
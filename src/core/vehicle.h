#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <string>
#include <vector>
#include "worldmap.h"
#include "pathplanner.h"
#include "behaviorplanner.h"

using std::string;
using std::vector;
using std::tuple;

class Vehicle {
    public:
        Vehicle() {

        }

        vector<double> getGlobalPosition() {
            return null;
        }

        vector<double> getFrenetPosition() {
            return null;
        }

        vector<double> getVelocity() {
            return null;
        }

private:
        double x;
        double y;

        double s;
        double dx;
        double dy;

        double vx;
        double vy;
};


#endif
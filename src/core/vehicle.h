#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <string>
#include <vector>

using std::string;
using std::vector;

class Vehicle {
    public:
        Vehicle() {

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
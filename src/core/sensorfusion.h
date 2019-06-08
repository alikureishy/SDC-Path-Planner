#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <iostream>
#include <string>
#include <vector>
#include "trackedvehicle.h"

using std::string;
using std::vector;

class SensorFusion {
    public:
        virtual vector<TrackedVehicle> getSurroundingVehicles() = 0;
    private:
        
};


#endif
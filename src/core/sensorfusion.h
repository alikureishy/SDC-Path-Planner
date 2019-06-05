#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <iostream>
#include <string>
#include <vector>

using std::string;
using std::vector;

class BaseSensorFusion {
    public:
        virtual vector<TrackedVehicle> getSurroundingVehicles() = 0;
    private:
        
};


#endif
#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <iostream>
#include <string>
#include <vector>

using std::string;
using std::vector;

// Sensor Fusion Indices:
#define SENSOR_ID 0
#define SENSOR_X 1
#define SENSOR_Y 2
#define SENSOR_VX 3
#define SENSOR_VY 4
#define SENSOR_S 5
#define SENSOR_D 6

class TrackedVehicle {
public:
    TrackedVehicle(nlohmann::json vehicle_data) {
        this->id = vehicle_data[SENSOR_ID];
        this->x = vehicle_data[SENSOR_X];
        this->y = vehicle_data[SENSOR_Y];
        this->vx = vehicle_data[SENSOR_VX];
        this->vy = vehicle_data[SENSOR_VY];
        this->speed = sqrt(vx*vx + vy*vy);
        this->s = vehicle_data[SENSOR_S];
        this->d = vehicle_data[SENSOR_D];
    }

    double predict_s(double time) {
        return (this->s + (time * this->speed));
    }

    int getId() const {
        return id;
    }

    double getX() const {
        return x;
    }

    double getY() const {
        return y;
    }

    double getVx() const {
        return vx;
    }

    double getVy() const {
        return vy;
    }

    double getSpeed() const {
        return speed;
    }

    double getS() const {
        return s;
    }

    double getD() const {
        return d;
    }

private:
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double speed;
    double s;
    double d;

};

class Environment {
public:
    Environment(nlohmann::json sensor_data) {
        this->numCars = sensor_data.size();
        for (int i = 0; i<numCars; i++) {
            vehicles.push_back(TrackedVehicle(sensor_data[i]));
        }
    }

    int getNumCars() const {
        return numCars;
    }

    const vector<TrackedVehicle>& getVehicles() const {
        return vehicles;
    }


private:
    int numCars;
    vector<TrackedVehicle> vehicles;
};


#endif
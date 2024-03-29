#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <iostream>
#include <string>
#include <vector>
#include "tgmath.h"
#include "constants.h"
#include "../3rdparty/json.hpp"

using std::string;
using std::vector;
using std::cout;
using std::endl;
using nlohmann::json;

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
    static bool isValid(json vehicle_data) {
        if (MIN_D < vehicle_data[SENSOR_D] && vehicle_data[SENSOR_D] < MAX_D) {
            return true;
        } else {
            return false;
        }
    }

    TrackedVehicle(json vehicle_data) {
        this->id = vehicle_data[SENSOR_ID];
        this->x = vehicle_data[SENSOR_X];
        this->y = vehicle_data[SENSOR_Y];
        this->vx = vehicle_data[SENSOR_VX];
        this->vy = vehicle_data[SENSOR_VY];
        this->speed = sqrt(vx*vx + vy*vy);
        this->s = vehicle_data[SENSOR_S];
        this->d = vehicle_data[SENSOR_D];
    }

    void fastForward(double time) {
        double increment = (time * this->speed);
        this->s += increment;
//        cout << "[------>] Fast forwarding vehicle id : " << this->id << " by " << time << " s = " << increment << "m" << endl;
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

    float getD() const {
        return d;
    }

    void setS(double s) {
        this->s = s;
    }

    int getLane() const {
        return floor(this->d / LANE_SIZE);
    }

private:
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double speed;
    double s;
    float d;

};

class Environment {
public:
    Environment(json sensor_data) {
        int invalid_count = 0;
        for (int i = 0; i<sensor_data.size(); i++) {
            if (TrackedVehicle::isValid(sensor_data[i])) {
                vehicles.push_back(TrackedVehicle(sensor_data[i]));
            } else {
                if (DEBUG_TELEMETRY) {
                    cout << "\t[TELEMETRY] Skipping vehicle " << sensor_data[i][SENSOR_ID] << " with D: " << sensor_data[i][SENSOR_D] << endl;
                }
                invalid_count++;
            }
        }

        if (invalid_count > 0) {
            if (DEBUG_TELEMETRY) {
                cout << "\t[TELEMETRY]Improper vehicle telemtry: " << invalid_count << +" vehicles" << endl;
            }
        }
    }

    int getNumCars() const {
        return vehicles.size();
    }

    const vector<TrackedVehicle>& getVehicles() const {
        return vehicles;
    }

    void fastForward(double seconds) {
        for (TrackedVehicle vehicle: this->vehicles) {
            vehicle.fastForward(seconds);
        }
    }

private:
    vector<TrackedVehicle> vehicles;
};


#endif // ENVIRONMENT_H
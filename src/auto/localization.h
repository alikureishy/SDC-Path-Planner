//
// Created by Admin on 2019-06-04.
//

#ifndef PATH_PLANNING_LOCALIZATION_H
#define PATH_PLANNING_LOCALIZATION_H

#include "constants.h"
#include "tgmath.h"
#include "../3rdparty/json.hpp"

using nlohmann::json;

class Localization {
public:
    Localization(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, double end_path_s, double end_path_d, json previous_path_x, json previous_path_y) {
        this->x = car_x;
        this->y = car_y;
        this->s = car_s;
        this->d = car_d;
        this->yaw = car_yaw;
        this->speed = car_speed;
        this->end_path_s = end_path_s;
        this->end_path_d = end_path_d;
        for (int i = 0; i<previous_path_x.size(); i++) {
            this->trajectory_x.push_back(previous_path_x[i]);
            this->trajectory_y.push_back(previous_path_y[i]);
        }

        if (this->trajectory_x.size() > 0) {
            this->s = this->end_path_s; // Pick up the last value of s from the trajectory being followed
        }

        // Calculate the current lane:
        this->lane = floor(this->d / LANE_SIZE);
    }

    double getX() const {
        return x;
    }

    double getY() const {
        return y;
    }

    double getS() const {
        return s;
    }

    double getD() const {
        return d;
    }

    double getYaw() const {
        return yaw;
    }

    double getSpeed() const {
        return speed;
    }

    const vector<double> &getTrajectoryX() const {
        return trajectory_x;
    }

    const vector<double> &getTrajectoryY() const {
        return trajectory_y;
    }

    void setS(double s) {
        this->s = s;
    }

    double getEndPathS() const {
        return end_path_s;
    }

    double getEndPathD() const {
        return end_path_d;
    }

    int getLane() const {
        return lane;
    }

private:
    int lane;
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    double end_path_s;
    double end_path_d;
    vector<double> trajectory_x;
    vector<double> trajectory_y;
};

#endif //PATH_PLANNING_LOCALIZATION_H

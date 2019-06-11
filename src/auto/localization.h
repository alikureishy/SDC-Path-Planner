//
// Created by Admin on 2019-06-04.
//

#ifndef PATH_PLANNING_LOCALIZATION_H
#define PATH_PLANNING_LOCALIZATION_H

#include "constants.h"
#include "tgmath.h"
#include "../3rdparty/json.hpp"
#include "worldmap.h"

using nlohmann::json;

class Localization {
public:
    Localization(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, double end_path_s, double end_path_d, json previous_path_x, json previous_path_y, WorldMap world_map)
            : world_map(world_map) {
        this->x = car_x;
        this->y = car_y;
        this->yaw = car_yaw;
        this->speed = car_speed;
        this->s = car_s;
        this->d = car_d;
        this->end_path_s = end_path_s;
        this->end_path_d = end_path_d;

        for (int i = 0; i<previous_path_x.size(); i++) {
            this->prev_trajectory_x.push_back(previous_path_x[i]);
            this->prev_trajectory_y.push_back(previous_path_y[i]);
        }

//        if (this->prev_trajectory_x.size() > 0) {
//            this->s = this->end_path_s; // Pick up the last value of s from the trajectory being followed
//            this->d = this->end_path_d;
//        }
    }

    void fastForward() {
        int steps = this->prev_trajectory_x.size();
        if (steps > 0) {
            this->x = this->prev_trajectory_x[steps-1];
            this->y = this->prev_trajectory_y[steps-1];
            double timelapse = steps * TIME_STEP;
            double increment = this->speed * timelapse;
            this->s+= increment;  // TODO: Need to convert x/y to s/d to get accurate value for d
                                                                    // Right now it's just a projection
            cout << "[======>] Fast forwarding ego by " << timelapse << " seconds ( " << steps << " steps of frenet-S = " << increment << "m)" << endl;
        }
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

    const vector<double> &getPrevTrajectory() const {
        return prev_trajectory_x;
    }

    const vector<double> &getPrevTrajectoryY() const {
        return prev_trajectory_y;
    }

    int getLane() const {
        return floor(this->d / LANE_SIZE);
    }

private:
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    double end_path_s;
    double end_path_d;
    vector<double> prev_trajectory_x;
    vector<double> prev_trajectory_y;
    WorldMap world_map;
};

#endif //PATH_PLANNING_LOCALIZATION_H

#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>
#include "localization.h"
#include "environment.h"

using std::tuple;

#define TIME_STEP 0.02 // seconds
#define PATH_LENGTH 30 // meters
#define PATH_POINTS 50
#define MAX_VELOCITY 49.5
#define MIN_VELOCITY 0.0
#define SPEED_INCREMENT 0.224
#define SPEED_DECREMENT 0.224
#define TAILGATE_GAP 40 // s units
#define LANE_SIZE 4 // meters
#define LANE_MIDPOINT (int)(LANE_SIZE/2) // meters

class TargetBehavior {
public:
    TargetBehavior(int lane, double speed) {
        this->lane = lane;
        this->speed = speed;
    }

    int getLane() const {
        return lane;
    }

    double getSpeed() const {
        return speed;
    }

private:
    int lane;
    double speed;
};

class BehaviorPlanner {
public:
    BehaviorPlanner () {}

    TargetBehavior planBehavior(const Localization& localization, const Environment& environment) const {

    }
};




#endif
//
// Created by Admin on 2019-06-04.
//

#ifndef PATH_PLANNING_AUTONOMOUS_H
#define PATH_PLANNING_AUTONOMOUS_H

#include "vehicle.h"

using std::shared_ptr;

class AutonomousVehicle {
    private:
        WorldMap_ worldMap;
        BehaviorPlanner_ behaviorPlanner;
        PathPlanner_ pathPlanner;

    public:
        AutonomousVehicle(WorldMap_ world, BehaviorPlanner_ behaviorPlanner, PathPlanner_ pathPlanner) {
            this->worldMap = world;
            this->behaviorPlanner = behaviorPlanner;
            this->pathPlanner = pathPlanner;
        }

        tuple<vector<double>, vector<double>> getTrajectory(const SensorFusion& sensorFusion, const Localization& localization, const NavigationContext& navigationContext);
};
typedef shared_ptr<AutonomousVehicle> AutonomousVehicle_;

#endif //PATH_PLANNING_AUTONOMOUS_H

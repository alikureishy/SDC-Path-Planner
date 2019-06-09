#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include "worldmap.h"

using std::tuple;
using std::make_tuple;
using std::shared_ptr;

class Trajectory {
public:
    Trajectory (const vector<double>& x_points, const vector<double>& y_points) : x_points(x_points), y_points(y_points) {}

    const vector<double> &getXPoints() const {
        return x_points;
    }

    const vector<double> &getYPoints() const {
        return y_points;
    }

private:
    vector<double> x_points;
    vector<double> y_points;
};

class PathPlanner {
public:
    PathPlanner(const WorldMap& world_map) : world_map(world_map) {}

    Trajectory planTrajectory(const TargetBehavior& target_behavior, const Localization& localization) const {

    }

private:
    WorldMap world_map;
};

//
//class StationaryPathPlanner : public PathPlanner {
//    public:
//        virtual tuple<vector<double>, vector<double>> plan() {
//            vector<double> empty_xs;
//            vector<double> empty_ys;
//            return make_tuple(empty_xs, empty_ys);
//        }
//};
//
//
//class StraightPathPlanner : public PathPlanner {
//public:
//        StraightPathPlanner(double speed, int numIntervals, WorldMap_ map) {   // # of 0.2 second intervals
//            this->speed = speed;
//            this->numIntervals = numIntervals;
//        }
//        virtual tuple<vector<double>, vector<double>> plan(int targetLane, double targetVelocity) {
//            vector<double> next_xs;
//            vector<double> next_ys;
//            double dist_inc = 0.5;
//            for(int i = 0; i < this->numIntervals; i++) {
//                double next_s = car_s + (i+1) * dist_inc;
//                double next_d = 6; // Center of middle-lane
//                vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//
//                next_xs.push_back(xy[0]);
//                next_ys.push_back(xy[1]);
//            }
//            return make_tuple(next_xs, next_ys);
//        }
//
//private:
//        double speed;
//        int numIntervals;
//};
//
//class CircularPathPlanner : public PathPlanner {
//    public:
//
//
//};


#endif
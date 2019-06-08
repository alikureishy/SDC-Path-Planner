#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

using std::tuple;
using std::make_tuple;
using std::shared_ptr;

class PathPlanner {
public:
    virtual tuple<vector<double>, vector<double>> plan() = 0;
};
typedef shared_ptr<PathPlanner> PathPlanner_;


class StationaryPathPlanner : public PathPlanner {
    public:
        virtual tuple<vector<double>, vector<double>> plan() {
            vector<double> empty_xs;
            vector<double> empty_ys;
            return make_tuple(empty_xs, empty_ys);
        }
};


class StraightPathPlanner : public PathPlanner {
public:
        StraightPathPlanner(double speed, int numIntervals) {   // # of 0.2 second intervals
            this->speed = speed;
            this->numIntervals = numIntervals;
        }
        virtual tuple<vector<double>, vector<double>> plan() {
            vector<double> next_xs;
            vector<double> next_ys;
            double dist_inc = 0.5;
            for(int i = 0; i < this->numIntervals; i++) {
                double next_s = car_s + (i+1) * dist_inc;
                double next_d = 6; // Center of middle-lane
                vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                next_xs.push_back(xy[0]);
                next_ys.push_back(xy[1]);
            }
            return make_tuple(next_xs, next_ys);
        }

private:
        double speed;
        int numIntervals;
};

class CircularPathPlanner : public PathPlanner {
    public:


};


#endif
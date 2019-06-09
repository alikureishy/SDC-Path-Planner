#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include "../3rdparty/spline.h"
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
    PathPlanner(const WorldMap& world_map) : invocation_counter(0), world_map(world_map) {}

    Trajectory planTrajectory(const TargetBehavior& target_behavior, const Localization& localization) {
        // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
        // Later we will interpolate these waypoints with a spline and fill it in with more points to control speed
        vector<double> spline_points_x;
        vector<double> spline_points_y;
        Localization ego(localization);

        // Reference x, y and yaw states
        // Either we will reference the starting point as where the car is, or at the previous path's end point
        double ref_x = ego.getX();
        double ref_y = ego.getY();
        double ref_yaw = deg2rad(ego.getYaw());

        // If previous size is almost empty, use the car current location as starting reference:
        int prev_trajectory_size = ego.getPrevTrajectory().size();
        if (prev_trajectory_size < 2) {
            // use 2 points that make the path tangent to the car
            double prev_car_x = ego.getX() - cos(ego.getYaw());
            double prev_car_y = ego.getY() - sin(ego.getYaw());

            spline_points_x.push_back(prev_car_x);
            spline_points_x.push_back(ego.getX());

            spline_points_y.push_back(prev_car_y);
            spline_points_y.push_back(ego.getY());
        } else {
            // use the previous path's end point as starting reference
            ref_x = ego.getPrevTrajectory()[prev_trajectory_size-1];
            ref_y = ego.getPrevTrajectoryY()[prev_trajectory_size-1];

            double ref_x_prev = ego.getPrevTrajectory()[prev_trajectory_size-2];
            double ref_y_prev = ego.getPrevTrajectoryY()[prev_trajectory_size-2];
            ref_yaw = atan2((ref_y-ref_y_prev), (ref_x-ref_x_prev));

            // use two points that make the path tanagent to the two previous path's end points:
            spline_points_x.push_back(ref_x_prev);
            spline_points_x.push_back(ref_x);

            spline_points_y.push_back(ref_y_prev);
            spline_points_y.push_back(ref_y);
        }


        // In frenet, add evenly 30m spaced poitns ahead of the starting reference:
        int target_lane = target_behavior.getLane();
        vector<double> next_wp0 = getXY(ego.getS() + 30, (LANE_MIDPOINT + (LANE_SIZE * target_lane)), world_map.getWaypointsS(), world_map.getWaypointsX(), world_map.getWaypointsY());
        vector<double> next_wp1 = getXY(ego.getS() + 60, (LANE_MIDPOINT + (LANE_SIZE * target_lane)), world_map.getWaypointsS(), world_map.getWaypointsX(), world_map.getWaypointsY());
        vector<double> next_wp2 = getXY(ego.getS() + 90, (LANE_MIDPOINT + (LANE_SIZE * target_lane)), world_map.getWaypointsS(), world_map.getWaypointsX(), world_map.getWaypointsY());

        spline_points_x.push_back(next_wp0[0]);
        spline_points_x.push_back(next_wp1[0]);
        spline_points_x.push_back(next_wp2[0]);

        spline_points_y.push_back(next_wp0[1]);
        spline_points_y.push_back(next_wp1[1]);
        spline_points_y.push_back(next_wp2[1]);

        // Shift frame to 0 degrees for all the widely spaced points:
        for (int i = 0; i < spline_points_x.size() ; i++) {
            double shift_x = spline_points_x[i] - ref_x;
            double shift_y = spline_points_y[i] - ref_y;

            spline_points_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            spline_points_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
        }


        // Fit a spline to the wide waypoints, to generate the finer-grained points
        tk::spline spline;
        spline.set_points(spline_points_x, spline_points_y);

        // Actual (x,y) points to be driven (returned to the simulator)
        vector<double> next_x_vals;
        vector<double> next_y_vals;


        // First, copy over all the previous path points from the last time
        for (int i = 0; i<prev_trajectory_size; i++) {
            next_x_vals.push_back(ego.getPrevTrajectory()[i]);
            next_y_vals.push_back(ego.getPrevTrajectoryY()[i]);
        }

        double target_x = PATH_LENGTH;
        double target_y = spline(target_x);
        double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

        double x_add_on = 0;

        // Fill up the REMAINING planned path after filling it with previous points, here we will always output 50 points:
        for(int i = 1; i<= (PATH_POINTS - prev_trajectory_size); i++) {
            double N = (target_dist / (TIME_STEP * target_behavior.getSpeed()/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = spline(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Rotate back to normal after rotating earlier:
            x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
        }

        this->invocation_counter++;

        return Trajectory(next_x_vals, next_y_vals);
    }

private:
    int invocation_counter;
    WorldMap world_map;
};

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
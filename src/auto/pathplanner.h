#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include "../3rdparty/spline.h"
#include "worldmap.h"
#include "math.h"

using std::tuple;
using std::make_tuple;
using std::shared_ptr;
using std::max;

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

    Trajectory planTrajectory(const StateMachine& target_behavior, const Localization& localization) {
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

        if (DEBUG_PATH_PLANNER) {
            cout << "\t[PATH_PLANNING] - Target-Lane : " << target_behavior.getLane() << " - Target-Speed : " << target_behavior.getSpeed() << endl;
        }
        // If previous size is almost empty, use the car current location as starting reference:
        int prev_trajectory_size = ego.getPrevTrajectoryX().size();
        if (prev_trajectory_size < 2) {
            // use 2 points that make the path tangent to the car
            double prev_car_x = ego.getX() - cos(ego.getYaw());
            double prev_car_y = ego.getY() - sin(ego.getYaw());

            spline_points_x.push_back(prev_car_x);
            spline_points_x.push_back(ego.getX());

            spline_points_y.push_back(prev_car_y);
            spline_points_y.push_back(ego.getY());

            if (DEBUG_PATH_PLANNER) {
                cout << "\t\t[START_PATH] : Last: (" << ego.getX() << ", " << ego.getY() << ") - Before-Last (" << prev_car_x << ", " << prev_car_y << ")" << endl;
            }
        } else {
            // use the previous path's end point as starting reference
            ref_x = ego.getPrevTrajectoryX()[prev_trajectory_size-1];
            ref_y = ego.getPrevTrajectoryY()[prev_trajectory_size-1];

            double ref_x_prev = ego.getPrevTrajectoryX()[prev_trajectory_size-2];
            double ref_y_prev = ego.getPrevTrajectoryY()[prev_trajectory_size-2];
            ref_yaw = atan2((ref_y-ref_y_prev), (ref_x-ref_x_prev));

            // use two points that make the path tanagent to the two previous path's end points:
            spline_points_x.push_back(ref_x_prev);
            spline_points_x.push_back(ref_x);

            spline_points_y.push_back(ref_y_prev);
            spline_points_y.push_back(ref_y);

            if (DEBUG_PATH_PLANNER) {
                cout << "\t\t[CONT_PATH] : Last: (" << ref_x << ", " << ref_y << ") - Before-Last (" << ref_x_prev << ", " << ref_y_prev << ")" << endl;
            }
        }

        // Add anchor points ahead of the starting reference:
        int current_lane = ego.getLane();
        int target_lane = target_behavior.getLane();
        double latitudinal_shift = (target_lane - current_lane) * LANE_SIZE; // -ve means left, +ve means right
        double latitudinal_shift_inc = latitudinal_shift / NUM_TRAJECTORY_ANCHORS;
        double longitudinal_shift = max<double>(TRAJECTORY_HORIZON, LANE_CHANGE_DURATION * target_behavior.getSpeed());
        double longitudinal_shift_inc = abs(longitudinal_shift / NUM_TRAJECTORY_ANCHORS);

        if (DEBUG_PATH_PLANNER) {
            cout << "\t\t[ANCHORS] - D_Shift = " << latitudinal_shift_inc << " (" << latitudinal_shift << "/" << NUM_TRAJECTORY_ANCHORS << ")"
                 << " - ^_Shift = " << latitudinal_shift_inc << " (" << latitudinal_shift << "/" << NUM_TRAJECTORY_ANCHORS << ")" << endl;
        }

        // We have to smooth out the latitudinal and longitudinal shift over a # of points.
        // Collect anchor points for that transition:
        for (int i = 0; i<NUM_TRAJECTORY_ANCHORS; i++) {
            double progressive_d_shift = i * latitudinal_shift_inc;
            double progressive_s_shift = (i+1) * longitudinal_shift_inc;
            vector<double> anchor = getXY(ego.getS() + progressive_s_shift, // S-shift
                                          LANE_MIDPOINT + (current_lane * LANE_SIZE) + progressive_d_shift, // D-shift
                                          world_map.getWaypointsS(),
                                          world_map.getWaypointsX(), // Worldmap-Xs
                                          world_map.getWaypointsY());

            // Add to spline
            spline_points_x.push_back(anchor[0]);   // x-coordinate
            spline_points_y.push_back(anchor[1]);   // y-coordinate
        }

        // Shift frame to 0 degrees for all the anchor points:
        for (int i = 0; i<spline_points_x.size(); i++) {
            // Translation
            double shift_x = spline_points_x[i] - ref_x;
            double shift_y = spline_points_y[i] - ref_y;

            // Shift frame to 0 degrees
            spline_points_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            spline_points_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

            if (DEBUG_ANCHORS) {
                cout << "\t\t\t[X/Y] - (" << spline_points_x[i] << ", " << spline_points_y[i] << ")" << endl;
            }
        }

        // Fit a spline to the anchor points, to generate the finer-grained points
        tk::spline spline;
        spline.set_points(spline_points_x, spline_points_y);

        // Actual (x,y) points to be driven (returned to the simulator)
        vector<double> next_x_vals;
        vector<double> next_y_vals;


        // First, copy over all the previous path points from the last time
        if (DEBUG_TRAJECTORY) {
            cout << "\t\t[TRAJECTORY]: (~) Prev Count: " << prev_trajectory_size << " - (*) Additional Count: " << NUM_TRAJECTORY_POINTS << endl;
        }
        for (int i = 0; i<prev_trajectory_size; i++) {
            next_x_vals.push_back(ego.getPrevTrajectoryX()[i]);
            next_y_vals.push_back(ego.getPrevTrajectoryY()[i]);
            if (DEBUG_TRAJECTORY) {
                cout << "\t\t\t~[X/Y] - (" << ego.getPrevTrajectoryX()[i] << ", " << ego.getPrevTrajectoryY()[i] << ")" << endl;
            }
        }

        double x_horizon = TRAJECTORY_HORIZON;
        double y_horizon = spline(x_horizon);
        double dist_horizon = sqrt((x_horizon*x_horizon) + (y_horizon*y_horizon));

        double x_add_on = 0;

        // Fill up the REMAINING planned path after filling it with previous points, here we will always output ALL points:
        for(int i = 1; i<= (NUM_TRAJECTORY_POINTS - prev_trajectory_size); i++) {
            double N = (dist_horizon / (TIME_STEP * target_behavior.getSpeed()/2.24));
            double x_point = x_add_on+(x_horizon)/N;
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

            if (DEBUG_TRAJECTORY) {
                cout << "\t\t\t*[X/Y] - (" << x_point << ", " << y_point << ")" << endl;
            }
        }

        return Trajectory(next_x_vals, next_y_vals);
    }

private:
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
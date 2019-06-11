//
// Created by safdar on 6/8/19.
//

#ifndef CONSTANTS_H
#define CONSTANTS_H

/**
 * CONSTANTS
 */
#define TIME_STEP 0.02                      // seconds DO NOT CHANGE!!!
#define MAX_SPEED 49.5                      // meters/second
#define MIN_SPEED 0.0                       // meters/second
#define NUM_LANES 3                         // units
#define LANE_SIZE 4                         // meters
#define LANE_MIDPOINT (int)(LANE_SIZE/2)    // meters

/**
 * PATH PLANNING
 */
#define LANE_CHANGE_DURATION 3              // seconds  -- This is a smoothing parameter
#define TRAJECTORY_HORIZON 15               // meters   -- This is the length of the trajectory we generate
#define NUM_TRAJECTORY_POINTS 10            // indices  -- This is the granularity of the trajectory

/**
 * BEHAVIORAL PLANNING
 */
#define SPEED_INCREMENT 0.224               // meters/second
#define SPEED_DECREMENT 0.224               // meters/second
#define SAFE_DRIVING_DISTANCE 30            // meters (frenet-S)

#define MIN_D -(NUM_LANES * LANE_SIZE)-1    //meters
#define MAX_D +(NUM_LANES * LANE_SIZE)+1    //meters

#define MIN_MERGE_OPENING 15                // meters
#define LANE_CHANGE_MORATORIUM 25           // cycles
#define CLEARANCE_ADVANTAGE_MULTIPLIER 10   // units


#endif // CONSTANTS_H

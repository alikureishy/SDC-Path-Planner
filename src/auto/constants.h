//
// Created by safdar on 6/8/19.
//

#ifndef CONSTANTS_H
#define CONSTANTS_H

#define TIME_STEP 0.02                      // seconds
#define TRAJECTORY_RANGE 30                 // meters
#define TRAJECTORY_LENGTH 50                // indices

#define MAX_SPEED 49.5                      // meters/second
#define MIN_SPEED 0.0                       // meters/second
#define SPEED_INCREMENT 0.224               // meters/second
#define SPEED_DECREMENT 0.224               // meters/second
#define SAFE_DRIVING_DISTANCE 20            // meters (frenet-S)

#define NUM_LANES 3                         // units
#define LANE_SIZE 4                         // meters
#define LANE_MIDPOINT (int)(LANE_SIZE/2)    // meters
#define MAX_D + (NUM_LANES * LANE_SIZE)     //
#define MIN_D - (NUM_LANES * LANE_SIZE)     //

#define MIN_LANE_OPENING 15                 // meters


#endif // CONSTANTS_H

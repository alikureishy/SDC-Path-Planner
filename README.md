# Model Documentation


## Building blocks

The following components work together to achieve the desired outcome

### Sensor Fusion

This allows the car to determine the location of all the vehicles around it, within a certain radius (horizon). This piece also has a predictive component to it, that's utilized by the behavior planner.

#### Behavior Prediction

This piece predicts the vehicle's position at the expected last point of the present trajectory being followed by the vehicle) -- i.e, in the future. The present prediction mechanism merely predicts the s-coordinate (frenet) using the vehicle's detected speed and current s-position. For the purposes of this project, this suffices.

### Localization

The localization piece takes the information presently known about the car, also predict the car's anticipated position at the end of the prior trajectory, and evaluate the right steps to take wrt the other cars' predicted behavior.

In this project, all we predict about other cars, and therefore also about ourselves, is the future s-coordinate. This proves sufficient to implement behavior and path planning. 


## Behavior Planning

This component is equivalent to the navigator that determines the best course of action at each point in time (after assessing a set of available options). It usually entails using a state machine appraoch, with a multitude of states that teh car can go through, as it navigates through traffic.

In this project, the states of the car are very trivial, but there is ample scope for adding additional states in the future.
Essentially, the car is always in one of two states:
- Changing lanes (or recently changed lanes), and
- Not changing lane 

Each time the behavior planner is invoked, it generates a target that consists of:

##### 1: Speed: Maintain safe distance from car in front:
    - Either slow down, stay the same speed or speed up to meet the maximum speed
##### 2: Lane: Target a lane change to the first lane (left to right) that satisfies the following criteria:
    - a merge space exists in that lane for the car to merge into
    - the merge would be advantageous to me (over a short-term horizon), and
    - it is safe (that is, the car ahead is not slowing down, and the car behind isn't speeding up)


## Path Planning (Trajectory Generation)

The path planner takes as input the behavior decided by the behavioral planner:
 - target speed, and 
 - target lane.

The path planner's job then, is simply to execute the above in the smoothest way possible. It achieves both via the following steps:
##### Step 1: Create a fixed # of anchors that perform the lane change (if any):
    - Ensure that any such lane change operation takes place over a distance equivalent to 3 seconds of longitudinal displacement (for smooth lane changes)
    - The first two anchors are the car's 2 prior positions until the present moment
    - The remaining anchros are based on the target speed and target lane
##### Step 2: Determine a spline that connects all those anchors
    - This allows us to then generate a finer-grained sequence of points to accelerate/decelerate the car 
##### Step 3: Use the spline to generate a sequence of points across the spline
    - These points are appropriately spaced apart to achieve the targeted speed
##### Step 4: Append new points to prior trajectory
    - This new trajectory is appened to the unused portion of the trajectory generated during the prior cycle, to ensure continuity and a natural smoothing across cycles.
    
## Configuration

The file constants.h contains the following configuration values that can be tweaked for better performance:

```
/**
 * Fixed cONSTANTS - !!!DO NOT CHANGE!!!
 */
#define TIME_STEP 0.02                      // seconds
#define MAX_SPEED 49.5                      // meters/second
#define MIN_SPEED 0.5                       // meters/second
#define NUM_LANES 3                         // units
#define LANE_SIZE 4                         // meters
#define LANE_MIDPOINT (int)(LANE_SIZE/2)    // meters

/**
 * PATH PLANNING
 */
#define LANE_CHANGE_DURATION 3              // seconds  -- This is a smoothing parameter
#define NUM_TRAJECTORY_ANCHORS 3            // units -- anchor points around which a function will be fitted
#define TRAJECTORY_HORIZON 20               // meters   -- This is the length of the trajectory we generate
#define NUM_TRAJECTORY_POINTS 10            // indices  -- This is the granularity of the trajectory

#define DEBUG_PATH_PLANNER false
#define DEBUG_ANCHORS false
#define DEBUG_TRAJECTORY false

/**
 * BEHAVIORAL PLANNING
 */
#define SPEED_INCREMENT 0.224               // meters/second
#define SPEED_DECREMENT 0.224               // meters/second
#define SAFE_DRIVING_DISTANCE 30            // meters (frenet-S)

#define MIN_D -(NUM_LANES * LANE_SIZE)-1    //meters
#define MAX_D +(NUM_LANES * LANE_SIZE)+1    //meters

#define MIN_MERGE_OPENING 15                // meters
#define LANE_CHANGE_MORATORIUM 200          // cycles (seconds)
#define CLEARANCE_ADVANTAGE_MULTIPLIER 3    // 300% percentage
#define SPEED_ADVANTAGE_MULTIPLIER 1.25     // 125% percentage

#define DEBUG_BEHAVIOR_PLANNER true
#define DEBUG_ENVIRONMENT_STATUS false
#define DEBUG_LANE_KEEP true
#define DEBUG_LANE_CHANGE true

/**
 * TELEMETRY
 */
#define DEBUG_TELEMETRY true

/**
 * LOCALIZATION
 */
#define DEBUG_LOCALIZATION true
```
 
## Future Enhancements

### Behavior Prediction
Improvements would invovle using Kalman filters, or such mechanisms, to predict not just the s-coordinate but also the d-coordinate, and to be able to make non-linear predictions about the same.

### Behavior Planning
Improvements would involve using an actual State Machine (perhaps using the Boost library for C++), and many states, that include:
- Keep lane
- Prepare to change lane (left or right)
- Changelane (left or right)


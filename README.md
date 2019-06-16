# Model Documentation


## Building blocks

The following components work together to achieve the desired outcome

### Sensor Fusion

This allows the car to determine the location of all the vehicles around it, within a certain radius (horizon). This piece also has a predictive component to it, that's utilized by the behavior planner.

#### Behavior Prediction

This piece predicts the vehicle's position at the expected last point of the present trajectory being followed by the vehicle) -- i.e, in the future. The present prediction mechanism merely predicts the s-coordinate (frenet) using the vehicle's detected speed and current s-position. There is ample scope for improvement here, but for the purposes of this project, this suffices. Later improvements could invovle using Kalman filters, or other mechanisms, to predict not just the s-coordinate but also the d-coordinate.  

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

#####1 Speed: Maintain safe distance from car in front:
    - Either slow down, stay the same speed or speed up to meet the maximum speed
#####2 Lane: Target a lane change to the first lane (left to right) that satisfies the following criteria:
    - a merge space exists in that lane for the car to merge into
    - the merge would be advantageous to me (over a short-term horizon), and
    - it is safe (that is, the car ahead is not slowing down, and the car behind isn't speeding up)


## Path Planning (Trajectory Generation)

The path planner takes as input the behavior decided by the behavioral planner:
 - target speed, and 
 - target lane.

The path planner's job then, is simply to execute the above in the smoothest way possible. It achieves both via the following steps:
#####1 Create a fixed # of anchors that perform the lane change (if any):
    - The first two anchors are the car's 2 prior positions until the present moment
    - The remaining anchros are based on the target speed and target lane
#####2 Determine a spline that connects all those anchors
    - This allows us to then generate a finer-grained sequence of points to accelerate/decelerate the car 
#####3 Use the spline to generate a sequence of points across the spline
    - These points are appropriately spaced apart to achieve the targeted speed
#####4 Append new points to prior trajectory
    - This new trajectory is appened to the unused portion of the trajectory generated during the prior cycle, to ensure continuity and a natural smoothing across cycles 
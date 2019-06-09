#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>
#include "localization.h"
#include "environment.h"
#include "constants.h"

using std::tuple;
using std::cout;
using std::endl;

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

    void setLane(int lane) {
        TargetBehavior::lane = lane;
    }

    void setSpeed(double speed) {
        TargetBehavior::speed = speed;
    }

private:
    int lane;
    double speed;
};

class BehaviorPlanner {
public:
    BehaviorPlanner (int initial_lane, double initial_speed) : invocation_counter(0), target_behavior(initial_lane, initial_speed) {}

    TargetBehavior planBehavior(const Localization& localization, const Environment& environment) {
        /* ============================================================== */
        /**
        * TODO: define a path made up of (x,y) points that the car will visit
        *   sequentially every .02 seconds
        */
        Localization ego(localization);
        cout << "Target speed: " << target_behavior.getSpeed() << " / Actual speed: " << ego.getSpeed() << endl;
//        assert (abs(target_behavior.getSpeed()-ego.getSpeed()) <= 1.0); // Actual speed should track closely with target speed
        double current_speed = this->target_behavior.getSpeed();    // Assume the car has achieved the target speed

        /**
         * BEHAVIOR PLANNER:
         * Outputs:
         *  - Target velocity
         *  - Lane #
         *
         * The path planner then is responsible for generating a trajectory using the above
         */
        bool unsafe_driving_distance = false;
        double closest_car_gap = SAFE_DRIVING_DISTANCE; // this will adjust to the distance of the closest car (if applicable)
        for (int i = 0; i<environment.getNumCars(); i++) {
            TrackedVehicle vehicle = environment.getVehicles()[i];

            if (ego.getLane() == vehicle.getLane()) {
                vehicle.fastForward(ego.getPrevTrajectory().size() * TIME_STEP); // Project the vehicle's position forward in time

                // Check s values greater than mine, that are closer than the acceptable s-gap:
                double distance = vehicle.getS() - ego.getS();
                if ((distance > 0) && ((distance < closest_car_gap))) {
                    closest_car_gap = distance;
                    cout << "Another car is too close: " << distance << "m" << endl;
                    unsafe_driving_distance = true;
                }
            }
        }

        // See if a lane-change is feasible and beneficial:
        //  - A lane change is beneficial if:
        //      - There is no car in that lane that is as close as the car in front
        //      - There is a car in that lane that is moving faster than the car in front




        // Reduce or increase the speed gradually
        if (unsafe_driving_distance && (current_speed > MIN_VELOCITY)) {
            double urgency = 1.0 - (closest_car_gap / SAFE_DRIVING_DISTANCE);
            assert (urgency >= 0.0 && urgency <= 1.0);
            double decrement = SPEED_DECREMENT*urgency;
            target_behavior.setSpeed(current_speed - decrement);
            cout << "[>] Slowing down car to vel " << current_speed << "m/s (! " << urgency << ")" << endl;
        } else if (current_speed < MAX_VELOCITY) {
            double urgency = 1.0 - (current_speed / MAX_VELOCITY);
            assert (urgency >= 0.0 && urgency <= 1.0);
            double increment = SPEED_INCREMENT*urgency;
            target_behavior.setSpeed(current_speed + increment);
            cout << "[>>>] Accelerating to vel " << current_speed << "m/s (! " << urgency << ")" << endl;
        }

        this->invocation_counter++;
        return this->target_behavior;
    }

private:
    int invocation_counter;
    TargetBehavior target_behavior;
};




#endif
#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>
#include <limits>
#include "localization.h"
#include "environment.h"
#include "constants.h"

using std::tuple;
using std::cout;
using std::endl;
using std::numeric_limits;

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

    /**
     * BEHAVIOR PLANNER:
     * Outputs:
     *  - Target velocity
     *  - Lane #
     *
     *  Conditions:
     *      - Too close to the car infront?
     *      - Is the car in front traveling slower than max speed limit?
     *          - Maybe move into the left lane?
     *              - Is there space to move into the left lane? (YES?)
     *                  - Scan space (+/- a window of s) in target lane to check this
     *                  - Is there a safe driving distance available to merge into in that lane (or some fraction of that gap)? (YES?)
     *              - Is the car further down that lane going at speed limit (or faster than the car in front of me)? (YES?)
     *              - Is the approaching car in that lane (within some fraction of safe driving distance) at same speed or slower than my speed? (YES?)
     *          -----> Then shift
     *          Else try right lane.
     *
     * The path planner then is responsible for generating a trajectory using the above
     */
    TargetBehavior planBehavior(const Localization& localization, const Environment& environment) {
        Localization ego(localization);
        double current_speed = this->target_behavior.getSpeed();    // Assume the car has achieved the target speed
        cout << "[STATUS] Lane: " << ego.getLane() << " / Target speed: " << target_behavior.getSpeed() << " / Actual speed: " << ego.getSpeed() << endl;

        /**
         * Evaluate current lane:
         *  - Are we at a safe driving distance in our current lane?
         *      - Reduce or increase the speed gradually
         */
        int evaluation_lane = ego.getLane();
        int closest_car_idx = getClosestCarInLane(environment, ego, evaluation_lane);
        double closest_car_gap = closest_car_idx >= 0 ?
                                        (abs(ego.getS() - environment.getVehicles()[closest_car_idx].getS()))
                                            :
                                        numeric_limits<double>::max();
        double closest_car_speed = closest_car_idx >= 0 ?
                                        environment.getVehicles()[closest_car_idx].getSpeed()
                                            :
                                        numeric_limits<double>::max();
        bool unsafe_driving_distance = closest_car_gap < SAFE_DRIVING_DISTANCE;
        if (unsafe_driving_distance && (current_speed > MIN_SPEED)) {
            double urgency = 1.0 - (closest_car_gap / SAFE_DRIVING_DISTANCE);
            assert (urgency >= 0.0 && urgency <= 1.0);
            double decrement = SPEED_DECREMENT * urgency;
            target_behavior.setSpeed(current_speed - decrement);
//            cout << "[COLLISION_AVOIDANCE]: [Gap: " << closest_car_gap << "] [>] Slowing down car to vel " << target_behavior.getSpeed() << "m/s (! " << urgency << ")" << endl;
        } else if (current_speed < MAX_SPEED) {
            double urgency = 1.0 - (current_speed / MAX_SPEED);
            assert (urgency >= 0.0 && urgency <= 1.0);
            double increment = SPEED_INCREMENT * urgency;
            target_behavior.setSpeed(current_speed + increment);
//            cout << "[SPEEDUP]: [>>>] Accelerating to vel " << target_behavior.getSpeed() << "m/s (! " << urgency << ")" << endl;
        } else {
//            cout << "[CRUISE_CONTROL]: [>>] Maintaining vel at " << target_behavior.getSpeed() << "m/s" << endl;
        }

        /**
         * If we're at an unsafe driving distance AND the car ahead is driving slower than the max speed:
         *  - Evaluate left & right lanes
         */
         bool nearing_unsafe_distance = closest_car_gap < 2 * SAFE_DRIVING_DISTANCE;
         bool we_could_go_faster = closest_car_speed < MAX_SPEED;
         if (nearing_unsafe_distance && we_could_go_faster) {
             // We evaluate the lane immediately to the left of the car, and then to the right of the car (unless it's at the edge)
             for (int evaluation_lane = std::max(0, ego.getLane()-1); evaluation_lane<=std::min(NUM_LANES-1, ego.getLane()+1); evaluation_lane++) {
                 if (evaluation_lane != ego.getLane()) {
                     // Front:
                     int closest_ahead_car_idx = getClosestCarInLane(environment, ego, evaluation_lane, false);
                     double closest_ahead_car_gap = closest_ahead_car_idx >= 0 ?
                                                    (abs(ego.getS() -
                                                         environment.getVehicles()[closest_ahead_car_idx].getS()))
                                                                               :
                                                    numeric_limits<double>::max();
                     double closest_ahead_car_speed = closest_ahead_car_idx >= 0 ?
                                                      environment.getVehicles()[closest_ahead_car_idx].getSpeed()
                                                                                 :
                                                      numeric_limits<double>::max();

                     // Rear:
                     int closest_behind_car_idx = getClosestCarInLane(environment, ego, evaluation_lane, true);
                     double closest_behind_car_gap = closest_behind_car_idx >= 0 ?
                                                     (abs(ego.getS() -
                                                          environment.getVehicles()[closest_behind_car_idx].getS()))
                                                                                 :
                                                     numeric_limits<double>::max();
                     double closest_behind_car_speed = closest_behind_car_idx >= 0 ?
                                                       environment.getVehicles()[closest_behind_car_idx].getSpeed()
                                                                                   :
                                                       numeric_limits<double>::min();

                     // Determine mergeability:
                     double slot_size = std::min(closest_ahead_car_gap, closest_behind_car_gap) * 2;

                     cout << "[EVALUATE_LANE_CHANGE --> " << evaluation_lane
                                << "] Slot-Size: " << slot_size
                                    << " / ^ " << closest_ahead_car_gap << "/" << closest_ahead_car_speed
                                    << " / v " << closest_behind_car_gap << "/" << closest_behind_car_speed
                            << endl;

                     // Determine merge advantage:
                     //     Switch lanes if:
                     //         - We ahve an opening AND
                     //         - It is advantageous relative to the current lane:
                     //             - Car in that lane is faster than us
                     //             OR
                     //             - Gap with that car is less than gap with our present lane's car
                     //                 AND
                     //               The car in that lane is traveling faster than the car in our lane
                     if (slot_size >= MIN_LANE_OPENING &&
                             (closest_ahead_car_speed > ego.getSpeed() ||
                             ((closest_ahead_car_gap < closest_car_gap) &&
                               closest_ahead_car_speed > closest_car_speed))) {
                         this->target_behavior.setLane(evaluation_lane);
                         cout << "[LANE_CHANGE] : [<<>>] Shifting to lane # " << evaluation_lane << endl;

                         // If the car behind is gaining in, increase spead:
                         double speed_diff = closest_behind_car_speed - current_speed;
                         if (speed_diff > 0) {
                             double urgency = std::min(1.0, (speed_diff / MAX_SPEED));
                             double increment = SPEED_INCREMENT * urgency;
                             target_behavior.setSpeed(current_speed + increment);
                             cout << "[LANE_CHANGE] [>>>] Accelerating to vel " << target_behavior.getSpeed()
                                  << "m/s (! " << urgency << ")" << endl;
                         }
                         break;
                     }
                 }
             }
         }


        this->invocation_counter++;
        return this->target_behavior;
    }

    /**
     * Returns the index of the closest car ahead of us (this is NOT the same number as the car-ID)
     *
     * @param environment
     * @param ego
     * @param target_lane
     * @return
     */
    int getClosestCarInLane(const Environment &environment, const Localization &ego, int target_lane, bool reverse = false) const {
        double closest_car_gap = numeric_limits<double>::max(); // maximum integer
        int closest_car_idx = -1;
        int direction = reverse ? -1 : 1;   // Controls which direction we're going to measure distance
        for (int i = 0; i < environment.getNumCars(); i++) {
            TrackedVehicle vehicle (environment.getVehicles()[i]);

            // TODO: Can we move this prediction step somewhere else that's cleaner?
            vehicle.fastForward(ego.getPrevTrajectory().size() * TIME_STEP); // Project the vehicle's position forward in time

            if (vehicle.getLane()==target_lane) {
                double distance = direction * (vehicle.getS() - ego.getS());    // Direction dictates forward or backward
                if ((distance > 0) && ((distance < closest_car_gap))) {
                    closest_car_gap = distance;
                    closest_car_idx = i;
                }
            }
        }
        return closest_car_idx;
    }

private:
    int invocation_counter;
    TargetBehavior target_behavior;


private:
};




#endif
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

class StateMachine {
public:
    StateMachine(int initial_lane, double initial_speed) {
        this->lane = initial_lane;
        this->speed = initial_speed;
        this->invocation_counter = 0;
        this->last_lane_change_cycle = 0;   // => No lane change possible until LANE_CHANGE_MORATORIUM is complete
    }

    int getLane() const {
        return lane;
    }

    double getSpeed() const {
        return speed;
    }

    void setLane(int lane) {
        StateMachine::lane = lane;
    }

    void setSpeed(double speed) {
        StateMachine::speed = speed;
    }

    void markLaneChanged() {
        last_lane_change_cycle = invocation_counter;
    }

    bool noRecentLaneChange() {
        bool result = invocation_counter - last_lane_change_cycle > LANE_CHANGE_MORATORIUM;
        return result;
    }

    void incrementInvocationCounter() {
        invocation_counter++;
    }

private:
    int lane;
    double speed;
    int invocation_counter;
    int last_lane_change_cycle;
};

class BehaviorPlanner {
public:
    BehaviorPlanner (int initial_lane, double initial_speed) : state_machine(initial_lane, initial_speed) {}

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
    StateMachine planBehavior(const Localization& localization_state, const Environment& environment_state) {
        Localization ego(localization_state);
        Environment environment(environment_state);

        // Fast-forward to end of existing trajectory (if applicable)
        ego.fastForward();
        environment.fastForward(ego.getPrevTrajectory().size() * TIME_STEP);    // Since we're APPENDING planned behavior to the trajectory


        /**
         * Evaluate current lane:
         *  - Are we at a safe driving distance in our current lane?
         *      - Reduce or increase the speed gradually
         */
        cout << "[STATUS] | " << ego.getLane() << " | with target speed: " << state_machine.getSpeed() << " / Actual speed: " << ego.getSpeed() << " / D: " << ego.getD() << endl;

        int evaluation_lane = ego.getLane();
        int closest_car_idx = getClosestCarInLane(environment, ego, evaluation_lane);
        double frontal_clearance = closest_car_idx >= 0 ?
                                        (abs(ego.getS() - environment.getVehicles()[closest_car_idx].getS()))
                                            :
                                        numeric_limits<double>::max();
        double frontal_car_speed = closest_car_idx >= 0 ?
                                        environment.getVehicles()[closest_car_idx].getSpeed()
                                            :
                                        numeric_limits<double>::max();
        bool unsafe_driving_distance = frontal_clearance < SAFE_DRIVING_DISTANCE;
        double current_speed_target = this->state_machine.getSpeed();    // Assume the car has achieved the target speed
        if (unsafe_driving_distance && (current_speed_target > MIN_SPEED)) {
            cout << "\t\t[ | XX | ] [--> " << frontal_clearance << "m -->]" << endl;
            double urgency = 1.0 - (frontal_clearance / SAFE_DRIVING_DISTANCE);
            assert (urgency >= 0.0 && urgency <= 1.0);
            double decrement = SPEED_DECREMENT * urgency;
            state_machine.setSpeed(current_speed_target - decrement);
            cout << "\t[vvvvv]: Slowing to velocity " << state_machine.getSpeed() << "m/s (Urgency: " << urgency << ")" << endl;
        } else if (current_speed_target < MAX_SPEED) {
            double urgency = 1.0 - (current_speed_target / MAX_SPEED);
            assert (urgency >= 0.0 && urgency <= 1.0);
            double increment = SPEED_INCREMENT * urgency;
            state_machine.setSpeed(current_speed_target + increment);
            cout << "\t[^^^^^]: Accelerating to vel " << state_machine.getSpeed() << "m/s (Urgency: " << urgency << ")" << endl;
        } else {
//            cout << "[CRUISE_CONTROL]: [>>] Maintaining vel at " << state_machine.getSpeed() << "m/s" << endl;
        }

        /**
         * If we're at an unsafe driving distance AND the car ahead is driving slower than the max speed:
         *  - Evaluate left & right lanes
         */
         bool nearing_unsafe_distance = frontal_clearance < 2 * SAFE_DRIVING_DISTANCE;
         bool we_could_go_faster = frontal_car_speed < MAX_SPEED;
         if (nearing_unsafe_distance && we_could_go_faster && this->state_machine.noRecentLaneChange()) {
             // We evaluate the lane immediately to the left of the car, and then to the right of the car (unless it's at the edge)
             for (int evaluation_lane = std::max(0, ego.getLane()-1); evaluation_lane<=std::min(NUM_LANES-1, ego.getLane()+1); evaluation_lane++) {
                 if (evaluation_lane != ego.getLane()) {
                     // Front:
                     int side_frontal_car_idx = getClosestCarInLane(environment, ego, evaluation_lane, false);
                     double side_frontal_opening = side_frontal_car_idx >= 0 ?
                                                    (abs(ego.getS() -
                                                         environment.getVehicles()[side_frontal_car_idx].getS()))
                                                                               :
                                                    numeric_limits<double>::max();
                     double side_frontal_car_speed = side_frontal_car_idx >= 0 ?
                                                      environment.getVehicles()[side_frontal_car_idx].getSpeed()
                                                                                 :
                                                      numeric_limits<double>::max();

                     // Rear:
                     int side_rear_car_idx = getClosestCarInLane(environment, ego, evaluation_lane, true);
                     double side_rear_opening = side_rear_car_idx >= 0 ?
                                                     (abs(ego.getS() -
                                                          environment.getVehicles()[side_rear_car_idx].getS()))
                                                                                 :
                                                     numeric_limits<double>::max();
                     double side_rear_car_speed = side_rear_car_idx >= 0 ?
                                                       environment.getVehicles()[side_rear_car_idx].getSpeed()
                                                                                   :
                                                       0;

                     // Determine mergeability:
                     double side_merge_opening = std::min(side_frontal_opening, side_rear_opening) * 2;

                     std::string shift_indicator;
                     if (evaluation_lane > ego.getLane()) {
                         shift_indicator = "|>>>>|";
                     } else if (evaluation_lane < ego.getLane()) {
                         shift_indicator = "|<<<<|";
                     }
                     cout << "[EVALUATE_LANE_CHANGE] [ |" << ego.getLane() << "| ==> |" << evaluation_lane << "|] " << ego.getSpeed() << "m/s" << endl
                                << "\t / |^^^^|"              << "^" << frontal_clearance << "m { " << frontal_car_speed << "m/s }" << endl
                                << "\t / " << shift_indicator << "Δ" << side_frontal_opening << "m { " << side_frontal_car_speed << "m/s }" << endl
                                << "\t / " << shift_indicator << "$" << side_merge_opening << "m" << endl
                                << "\t / " << shift_indicator << "V " << side_rear_opening << "m { " << side_rear_car_speed <<"m/s }" << endl;

                     // Determine merge advantage:
                     //     Switch lanes if:
                     //         - We ahve an opening AND
                     //         - It is advantageous relative to the current lane:
                     //             - Car in that lane is faster than us
                     //             OR
                     //             - Gap with that car is less than gap with our present lane's car
                     //                 AND
                     //               The car in that lane is traveling faster than the car in our lane
                     bool side_frontal_opening_allows_overtake = side_frontal_opening > frontal_clearance * CLEARANCE_ADVANTAGE_MULTIPLIER;

                     bool side_frontal_car_faster_than_me = side_frontal_car_speed > ego.getSpeed();
                     bool side_frontal_car_faster_than_car_ahead = side_frontal_car_speed >= frontal_car_speed;
                     bool side_rear_car_slower_than_me = side_rear_car_speed <= ego.getSpeed();

                     bool side_frontal_car_not_a_risk = side_frontal_opening >= MIN_LANE_OPENING || side_frontal_car_faster_than_me;
                     bool side_rear_car_not_a_risk = side_rear_opening >= MIN_LANE_OPENING || side_rear_car_slower_than_me;

                     bool merge_space_exists = side_merge_opening >= MIN_LANE_OPENING;
                     bool merge_is_advantageous = side_frontal_car_faster_than_car_ahead || side_frontal_opening_allows_overtake;
                     bool merge_is_safe = side_frontal_car_not_a_risk && side_rear_car_not_a_risk;

                     if (merge_space_exists && merge_is_advantageous && merge_is_safe) {
                         this->state_machine.setLane(evaluation_lane);
                         this->state_machine.markLaneChanged();
                         cout << "[LANE_CHANGE] : [<<>>] Shifting to lane # " << evaluation_lane << endl;

                         // If the car behind is gaining in, increase spead:
                         double speed_diff = side_rear_car_speed - current_speed_target;
                         if (speed_diff > 0) {
                             double urgency = std::min(1.0, (speed_diff / MAX_SPEED));
                             double increment = SPEED_INCREMENT * urgency;
                             state_machine.setSpeed(current_speed_target + increment);
                             cout << "[LANE_CHANGE] [>>>] Accelerating to vel " << state_machine.getSpeed()
                                  << "m/s (! " << urgency << ")" << endl;
                         }
                         break;
                     }
                 }
             }
         }


        this->state_machine.incrementInvocationCounter();
        return this->state_machine;
    }

    /**
     * Returns the index of the closest car ahead of us (this is NOT the same number as the car-ID)
     *
     * @param environment
     * @param ego
     * @param check_lane
     * @return
     */
    int getClosestCarInLane(const Environment &environment, const Localization &ego, int check_lane, bool reverse = false) const {
        double closest_car_gap = numeric_limits<double>::max(); // maximum integer
        int closest_car_idx = -1;
        int direction = reverse ? -1 : 1;   // Controls which direction we're going to measure distance
        for (int i = 0; i < environment.getNumCars(); i++) {
            TrackedVehicle vehicle (environment.getVehicles()[i]); // Make sure to use a copy here

            if (vehicle.getLane()==check_lane) {
                double distance = direction * (vehicle.getS() - ego.getS());    // Direction dictates forward or backward
                if ((distance > 0) && ((distance < closest_car_gap))) {
                    closest_car_gap = distance;
                    closest_car_idx = i;
                } else {
                }
                cout << "\t\t[ | ~~ | ] Vehicle ID: " << vehicle.getD() << " | " << vehicle.getLane() << " | " << distance << " m ahead (d= " << vehicle.getD() << ")" << endl;
            } else {
                cout << "\t\t[ ------ ] Vehicle ID: " << vehicle.getD() << " | " << vehicle.getLane() << " | at s= " << vehicle.getS() << " / d= " << vehicle.getD() << endl;
            }
        }
        return closest_car_idx;
    }

private:
    StateMachine state_machine;


private:
};




#endif
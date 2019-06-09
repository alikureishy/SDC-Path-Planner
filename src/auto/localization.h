//
// Created by Admin on 2019-06-04.
//

#ifndef PATH_PLANNING_LOCALIZATION_H
#define PATH_PLANNING_LOCALIZATION_H

class Localization {
public:
    Localization(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, double end_path_s, double end_path_d, vector<double> previous_path_x, vector<double> previous_path_y) {
        this->x = car_x;
        this->y = car_y;
        this->s = car_s;
        this->d = car_d;
        this->yaw = car_yaw;
        this->speed = car_speed;
        this->previous_path_x = previous_path_x;
        this->previous_path_y = previous_path_y;
    }

    double getX() const {
        return x;
    }

    double getY() const {
        return y;
    }

    double getS() const {
        return s;
    }

    double getD() const {
        return d;
    }

    double getYaw() const {
        return yaw;
    }

    double getSpeed() const {
        return speed;
    }

    const vector<double> &getPreviousPathX() const {
        return previous_path_x;
    }

    const vector<double> &getPreviousPathY() const {
        return previous_path_y;
    }

private:
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    vector<double> previous_path_x;
    vector<double> previous_path_y;
};

#endif //PATH_PLANNING_LOCALIZATION_H

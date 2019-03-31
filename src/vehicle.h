#ifndef SDC_VEHICLE_MODULE
#define SDC_VEHICLE_MODULE

#include <map>
#include <math.h>
#include "telemetry.h"
#include "props.h"

using std::map;

class Vehicle {
    private:
        int id;
        double x;
        double y;
        double vx;
        double vy;
        double s;
        double d;

        double speed;
        double yaw;

        map<int, Vehicle> other_vehicles;

    public:
        Vehicle(vector<double> data) {
            this->id = (int)data[0];
            this->x = data[1];
            this->y = data[2];
            this->vx = data[3];
            this->vy = data[4];
            this->s = data[5];
            this->d = data[6];

            if (this->vx == 0) {
                this->vx = 0.00001;
            }
            this->speed = sqrt(this->vx * this->vx + this->vy * this->vy);
            this->yaw = atan(this->vy / this->vx);
        }

        Vehicle() {}
        ~Vehicle() {}

        int get_id() {
            return id;
        }

        double get_x() {
            return this->x;
        }

        double get_y() {
            return this->y;
        }

        double get_vx() {
            return this->vx;
        }

        double get_vy() {
            return this->vy;
        }

        double get_s() {
            return this->s;
        }

        double get_d() {
            return this->d;
        }

        double get_speed() {
            return this->speed;
        }

        double get_yaw() {
            return this->yaw;
        }
};
#endif
#ifndef SDC_VEHICLE_MODULE
#define SDC_VEHICLE_MODULE

#include <map>
#include <iostream>
#include <algorithm> // min, max
#include <stdlib.h> // rand
#include "telemetry.h"
#include "props.h"
#include "helpers.h" // rad2deg

using std::map;

static const double MAX_ACCELERATION = 0.224;

class Vehicle {
    private:
        Props props;

        int id;
        double x;
        double y;
        double vx;
        double vy;
        double s;
        double d;

        double speed;
        double yaw;
        
        double ref_speed = 20.0;
        
        int uid = rand() % 1000;

    public:
        Vehicle(int id) {
            std::cout << "inside parameterised constructor of vehicle. uid: " << this->uid << std::endl;
            this->id = id;
        }

        Vehicle() {
            std::cout << "inside empty constructor of vehicle. uid: " << this->uid << std::endl;
            this->id = -1; // Ego vehicle
        }
        ~Vehicle() {
            std::cout << "inside deconstructor of vehicle. uid: " << this->uid << std::endl;
        }

        void update_sensor_data(vector<double> sensor_data) {
            std::cout << "updating sensor data" << std::endl;
            this->x = sensor_data[1];
            this->y = sensor_data[2];
            this->vx = sensor_data[3];
            this->vy = sensor_data[4];
            this->s = sensor_data[5];
            this->d = sensor_data[6];

            if (this->vx == 0) {
                this->vx = 0.00001;
            }
            this->speed = sqrt(this->vx * this->vx + this->vy * this->vy);
            this->yaw = atan(this->vy / this->vx);
        }

        void accelerate() {
            double speed_limit = props.speed_limit();
            std::cout << "before acceleration. speed: " << this->ref_speed << std::endl;
            this->ref_speed = std::min(this->ref_speed + MAX_ACCELERATION, speed_limit);
            std::cout << "after acceleration. speed: " << this->ref_speed << std::endl;
        }

        void slow_down() {
            double speed_limit = props.speed_limit();
            this->ref_speed = std::max(this->ref_speed - MAX_ACCELERATION, 0.0);
        }

        double get_ref_speed() {
            return this->ref_speed;
        }

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

        double get_yaw_in_degrees() {
            return rad2deg(this->get_yaw());
        }

        int get_lane() {
            double lane_width_in_meters = props.lane_width_in_meters();
            return (int)(this->get_d() / lane_width_in_meters);
        }

        // Todo: Use some templating later
        void print_vehicle_information() {
            string text = "***** Vehicle: " + std::to_string(get_id()) + " *****\n"
                + "x: " + std::to_string(get_x()) + ", y: " + std::to_string(get_y()) + ", vx: " + std::to_string(get_vx()) + ", vy: " + std::to_string(get_vy()) + ", s: " + std::to_string(get_s()) + ", d: " + std::to_string(get_d()) + "\n" +
                "speed: " + std::to_string(get_speed()) + ", yaw: " + std::to_string(get_yaw()) + ", yaw (degrees): " + std::to_string(get_yaw_in_degrees()) + ", ref speed: " + std::to_string(get_ref_speed()) + ", lane: " + std::to_string(get_lane()) + "\n\n" + '\0';
            std::cout << text << std::endl;
        }
};
#endif
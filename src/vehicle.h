#ifndef SDC_VEHICLE_MODULE
#define SDC_VEHICLE_MODULE

#include <map>
#include <vector>
#include <iostream>
#include <algorithm> // min, max
#include <stdlib.h> // rand
#include <cstdlib> // abs
#include "telemetry.h"
#include "props.h"
#include "helpers.h" // rad2deg

using std::map;
using std::vector;

static const double MAX_ACCELERATION = 0.224;
static const double MPH_TO_METERS_PER_SEC_CONV = 0.447038;

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
        
        double ref_speed = 0.0;
        double ref_lane = 1; // TODO: Unhardcode this!
        
        int uid = rand() % 1000;

    public:
        Vehicle(int id) {
            this->id = id;
        }

        Vehicle() {
            this->id = -1; // Ego vehicle
        }
        ~Vehicle() {
        }

        void update_sensor_data(vector<double> sensor_data) {
            this->x = sensor_data[1];
            this->y = sensor_data[2];
            this->vx = sensor_data[3];
            this->vy = sensor_data[4];
            this->s = sensor_data[5];
            this->d = sensor_data[6];

            if (this->vx == 0) {
                this->vx = 0.00001;
            }
            this->speed = this->get_speed();
            this->yaw = atan(this->vy / this->vx);
        }

        void accelerate() {
            double speed_limit = props.speed_limit();
            this->ref_speed = std::min(this->ref_speed + MAX_ACCELERATION, speed_limit);
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
            return sqrt(this->vx * this->vx + this->vy * this->vy);
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

        int get_ref_lane() {
            return this->ref_lane;
        }

        void prepare_lane_change_left() {
            this->ref_lane = this->get_lane() -1;
        }

        void prepare_lane_change_right() {
            this->ref_lane = this->get_lane() + 1;
        }

        // Todo: Use some templating later
        void print_vehicle_information() {
            string text = "***** Vehicle: " + std::to_string(get_id()) + " *****\n"
                + "x: " + std::to_string(get_x()) + ", y: " + std::to_string(get_y()) + ", vx: " + std::to_string(get_vx()) + ", vy: " + std::to_string(get_vy()) + ", s: " + std::to_string(get_s()) + ", d: " + std::to_string(get_d()) + "\n" +
                "speed: " + std::to_string(get_speed()) + ", yaw: " + std::to_string(get_yaw()) + ", yaw (degrees): " + std::to_string(get_yaw_in_degrees()) + ", ref speed: " + std::to_string(get_ref_speed()) + ", lane: " + std::to_string(get_lane()) + "\n\n" + '\0';
            std::cout << text << std::endl;
        }

        bool behind(Vehicle another_vehicle) {
            return another_vehicle.get_s() > this->get_s();
        }

        bool same_lane_as(Vehicle another_vehicle) {
            return another_vehicle.get_lane() == this->get_lane();
        }

        bool immediate_left_of(Vehicle another_vehicle) {
            return another_vehicle.get_lane() - this->get_lane() == 1;
        }

        bool immediate_right_of(Vehicle another_vehicle) {
            return this->get_lane() - another_vehicle.get_lane() == 1;
        }

        double distance(Vehicle another_vehicle) {
            return another_vehicle.get_s() - this->get_s(); // positive if the other vehicle is ahead, else negative
        }

        double abs_distance(Vehicle another_vehicle) {
            return abs(distance(another_vehicle)); // net distance with other vehicle; always positive
        }

        double closest_vehicle_ahead_distance(map<int, Vehicle> &vehicles) {
            double least_distance = 10000;
            for (int i = 0; i < vehicles.size(); i++) {
                Vehicle another_vehicle = vehicles[i];
                if (this->same_lane_as(another_vehicle) && this->behind(another_vehicle)) {
                    double distance = this->abs_distance(another_vehicle);
                    if (distance < least_distance) {
                        least_distance = distance;
                    }
                }
            }

            return least_distance;
        }

        double time_to_complete_lane_change() {
            double lane_change_distance = 2 * props.lane_switch_in_meters();
            double velocity_in_meters_per_sec = this->get_speed() * MPH_TO_METERS_PER_SEC_CONV;
            return lane_change_distance / velocity_in_meters_per_sec;
        }

        double closest_distance_during_lane_change(Vehicle &other_vehicle) {
            int lane_change_duration_ms = (int)(this->time_to_complete_lane_change() * 1000);
            int lapse = 0;
            int steps = 5;
            int closest_distance = 1000;
            for (int i = 0; i < steps; i++) {
                lapse += (lane_change_duration_ms / steps);
                double lapse_in_seconds = lapse / 1000;
                double this_s = this->get_s() + (this->get_speed() * MPH_TO_METERS_PER_SEC_CONV) * lapse_in_seconds; // s2 = s1 + v*dt
                double other_s = other_vehicle.get_s() + (other_vehicle.get_speed() * MPH_TO_METERS_PER_SEC_CONV) * lapse_in_seconds;
                double gap = abs(this_s - other_s);
                if (gap < closest_distance) {
                    closest_distance = gap;
                }
            }
            return closest_distance;
        }

        bool approaching(Vehicle &other_vehicle) {
            double duration = 1.0;
            double distance_now = abs(this->get_s() - other_vehicle.get_s());

            double this_s = this->get_s() + (this->get_speed() * MPH_TO_METERS_PER_SEC_CONV) * duration;
            double other_s = other_vehicle.get_s() + (other_vehicle.get_speed() * MPH_TO_METERS_PER_SEC_CONV) * duration;
            double distance_later = abs(this_s - other_s);

            return distance_later < distance_now;
        }
};
#endif
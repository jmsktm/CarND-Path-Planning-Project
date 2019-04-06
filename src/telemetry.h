#ifndef SDC_TELEMETRY_MODULE
#define SDC_TELEMETRY_MODULE

#define MPH_TO_METERS_PER_SECOND 0.44703888888

#include <tuple>
#include <string>
#include <vector>
#include <math.h> /* atan2 */
#include <map>
#include <iostream>
#include <algorithm> // std::min

#include "spline.h"
#include "helpers.h"
#include "props.h"
#include "map.h"
#include "vehicle.h"
#include "utils.h"

#include "json.hpp"
using nlohmann::json;

using std::string;
using std::tuple;
using std::vector;
using std::map;

static const double ACCELERATION = 0.224;
static const double COLLISION_COST = 500;

class Telemetry {
    private:
        Map geo_map;
        Props props;
        string str;
        json data;
        map<int, Vehicle> vehicle_map;

        Vehicle& ego_vehicle;
    public:
        ~Telemetry() {}

        vector<double> get_ego_vehicle_data() {
            if (this->_hasEventMessageData()) {
                json telemetry_data = this->_getTelemetryData();
                double vehicle_id = -1;
                double vehicle_x = telemetry_data.value("x", 0.0);
                double vehicle_y = telemetry_data.value("y", 0.0);
                double vehicle_yaw = telemetry_data.value("yaw", 0.0);
                double vehicle_speed = telemetry_data.value("speed", 0.0);
                double vehicle_s = telemetry_data.value("s", 0.0);
                double vehicle_d = telemetry_data.value("d", 0.0);

                double vehicle_vx = vehicle_speed * cos(vehicle_yaw);
                double vehicle_vy = vehicle_speed * sin(vehicle_yaw);

                return { vehicle_id, vehicle_x, vehicle_y, vehicle_vx, vehicle_vy, vehicle_s, vehicle_d };
            }
            return {};
        }

        void populate_vehicle_map() {
            vehicle_map.clear();
            if (this->_hasTelemetryData()) {
                json sensor_fusion = this->get_sensor_fusion();
                if (!sensor_fusion.empty()) {
                    for (int i = 0; i < sensor_fusion.size(); i++) {
                        vector<double> sensor_data = sensor_fusion[i];
                        if (!sensor_data.empty()) {
                            int vehicle_id = (int)sensor_data[0];
                            Vehicle vehicle = Vehicle(vehicle_id);
                            vehicle.update_sensor_data(sensor_data);
                            vehicle_map[vehicle_id] = vehicle;
                        }
                    }
                }
            }
        }

        map<int, Vehicle> get_surrounding_vehicles() {
            return vehicle_map;
        }

        Telemetry(string str, Vehicle &vehicle): ego_vehicle(vehicle) {
            this->str = str;
            if (this->_hasTelemetryData()) {
                data = this->_getTelemetryData();
                vector<double> ego_vehicle_data = this->get_ego_vehicle_data();
                if (ego_vehicle_data.size() > 0) {
                    this->ego_vehicle.update_sensor_data(ego_vehicle_data);
                }
                populate_vehicle_map();
            }
        }

        Vehicle get_ego_vehicle() {
            return this->ego_vehicle;
        }

        bool _hasEventMessageData() {
            // "42" at the start of the message means there's a websocket message event.
            // The 4 signifies a websocket message. The 2 signifies a websocket event
            if (this->str.size() > 2 && this->str[0] == '4' && this->str[1] == '2') {
                auto found_null = str.find("null");
                auto b1 = str.find_first_of("[");
                auto b2 = str.find_first_of("}");
                if (found_null == string::npos && b1 != string::npos && b2 != string::npos) {
                    return true;
                }
            }
            return false; 
        }

        string _getEventMessageData() {
            auto b1 = this->str.find_first_of("[");
            auto b2 = this->str.find_first_of("}");
            return this->str.substr(b1, b2-b1 + 2);
        }

        bool _hasTelemetryData() {
            if (!this->_hasEventMessageData()) {
                return false;
            }
            auto eventMessageData = this->_getEventMessageData();
            auto j = json::parse(eventMessageData);
            string event = j[0].get<string>();
            return event == "telemetry";
        }

        json _getTelemetryData() {
            if (!this->_hasTelemetryData()) {
                return json({});
            }
            auto eventMessageData = this->_getEventMessageData();
            auto j = json::parse(eventMessageData);
            return j[1];
        }

        json get_previous_path_x() {
            return data["previous_path_x"];
        }

        json get_previous_path_y() {
            return data["previous_path_y"];
        }

        double get_end_path_s() {
            return data["end_path_s"];
        }

        double get_end_path_d() {
            return data["end_path_d"];
        }

        json get_sensor_fusion() {
            return data["sensor_fusion"];
        }

        tuple<vector<double>, vector<double>> derive_last_two_points_from_current_pos() {
            double x = this->get_ego_vehicle().get_x();
            double y = this->get_ego_vehicle().get_y();
            double yaw = this->get_ego_vehicle().get_yaw();

            double pos2_x = x;
            double pos2_y = y;

            double pos1_x = x - cos(yaw);
            double pos1_y = y - sin(yaw);

            vector<double> ptsx;
            vector<double> ptsy;

            ptsx.push_back(pos1_x);
            ptsx.push_back(pos2_x);

            ptsy.push_back(pos1_y);
            ptsy.push_back(pos2_y);

            return std::make_tuple(ptsx, ptsy);
        }

        tuple<vector<double>, vector<double>> get_last_two_points_from_previous_path() {
            vector<double> previous_path_x = get_previous_path_x();
            vector<double> previous_path_y = get_previous_path_y();

            int prev_size = get_previous_path_x().size();
            
            double pos2_x = previous_path_x[prev_size - 1];
            double pos2_y = previous_path_y[prev_size - 1];

            double pos1_x = previous_path_x[prev_size - 2];
            double pos1_y = previous_path_y[prev_size - 2];

            vector<double> ptsx;
            vector<double> ptsy;

            ptsx.push_back(pos1_x);
            ptsx.push_back(pos2_x);

            ptsy.push_back(pos1_y);
            ptsy.push_back(pos2_y);

            return std::make_tuple(ptsx, ptsy);
        }

        tuple<vector<double>, vector<double>> get_next_3_evenly_spaced_waypoints(double gap, int target_lane) {
            double s = this->get_ego_vehicle().get_s();
            double target_d = props.get_d_by_lane(target_lane);

            vector<double> next_waypoint0 = getXY(s + 2 * gap, target_d, geo_map.get_waypoints_s(), geo_map.get_waypoints_x(), geo_map.get_waypoints_y());
            vector<double> next_waypoint1 = getXY(s + 3 * gap, target_d, geo_map.get_waypoints_s(), geo_map.get_waypoints_x(), geo_map.get_waypoints_y());
            vector<double> next_waypoint2 = getXY(s + 4 * gap, target_d, geo_map.get_waypoints_s(), geo_map.get_waypoints_x(), geo_map.get_waypoints_y());

            vector<double> ptsx = { next_waypoint0[0], next_waypoint1[0], next_waypoint2[0] };
            vector<double> ptsy = { next_waypoint0[1], next_waypoint1[1], next_waypoint2[1] };

            return std::make_tuple(ptsx, ptsy);
        }

        tuple<vector<double>, vector<double>> get_5_points_for_spline_generation(double gap, int target_lane) {
            int prev_size = get_previous_path_x().size();

            vector<double> ptsx;
            vector<double> ptsy;

            // Adding 2 previous points
            tuple<vector<double>, vector<double>> prev_points_tuple;
            if (prev_size < 2) {
                prev_points_tuple = derive_last_two_points_from_current_pos();
            } else {
                prev_points_tuple = get_last_two_points_from_previous_path();
            }
            vector<double> prev_vector_x = std::get<0>(prev_points_tuple);
            vector<double> prev_vector_y = std::get<1>(prev_points_tuple);

            ptsx.push_back(prev_vector_x[0]);
            ptsx.push_back(prev_vector_x[1]);

            ptsy.push_back(prev_vector_y[0]);
            ptsy.push_back(prev_vector_y[1]);

            // Adding 3 next points
            tuple<vector<double>, vector<double>> next_points_tuple = get_next_3_evenly_spaced_waypoints(gap, target_lane);

            vector<double> next_vector_x = std::get<0>(next_points_tuple);
            vector<double> next_vector_y = std::get<1>(next_points_tuple);

            ptsx.push_back(next_vector_x[0]);
            ptsx.push_back(next_vector_x[1]);
            ptsx.push_back(next_vector_x[2]);

            ptsy.push_back(next_vector_y[0]);
            ptsy.push_back(next_vector_y[1]);
            ptsy.push_back(next_vector_y[2]);

            return std::make_tuple(ptsx, ptsy);
        }

        tuple<vector<double>, vector<double>> get_coordinates_in_car_frame(tuple<vector<double>, vector<double>> &world_frame_coordinates) {
            vector<double> world_ptsx = std::get<0>(world_frame_coordinates);
            vector<double> world_ptsy = std::get<1>(world_frame_coordinates);

            double ref_x = world_ptsx[1];
            double ref_y = world_ptsy[1];

            double ref_x_prev = world_ptsx[0];
            double ref_y_prev = world_ptsy[0];

            double ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            vector<double> ptsx;
            vector<double> ptsy;

            for (int i = 0; i < world_ptsx.size(); i++) {
                double shift_x = world_ptsx[i] - ref_x;
                double shift_y = world_ptsy[i] - ref_y;

                double x = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                double y = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

                ptsx.push_back(x);
                ptsy.push_back(y);
            }

            return std::make_tuple(ptsx, ptsy);
        }

        /* Move the logics related to cost function and lane change to a separate file - start */
        int cost_left_lane() {
            int cost = 0;
            if (ego_vehicle.get_lane() == 0) {
                return COLLISION_COST;
            }
            for (int i = 0; i < this->vehicle_map.size(); i++) {
                Vehicle other_vehicle = vehicle_map[i];
                if (ego_vehicle.immediate_right_of(other_vehicle)) {
                    double closest_distance = ego_vehicle.closest_distance_during_lane_change(other_vehicle);
                    if (closest_distance < 10) {
                        if (ego_vehicle.approaching(other_vehicle)) {
                            return COLLISION_COST;
                        } else {
                            int current_cost = (int)((10.0 - closest_distance) * 10);
                            if (current_cost > cost) {
                                cost = current_cost;
                            }
                        }
                    }
                }
            }
            return 50 + cost; // 50 is the cost of performing lane change
        }

        int cost_right_lane() {
            int cost = 0;
            int rightmost_lane_index = props.current_road_lanes() - 1;
            if (ego_vehicle.get_lane() == rightmost_lane_index) {
                return COLLISION_COST;
            }
            for (int i = 0; i < this->vehicle_map.size(); i++) {
                Vehicle other_vehicle = vehicle_map[i];
                if (ego_vehicle.immediate_left_of(other_vehicle)) {
                    double closest_distance = ego_vehicle.closest_distance_during_lane_change(other_vehicle);
                    if (closest_distance < 10) {
                        if (ego_vehicle.approaching(other_vehicle)) {
                            return COLLISION_COST;
                        } else {
                            int current_cost = (int)((10.0 - closest_distance) * 10);
                            if (current_cost > cost) {
                                cost = current_cost;
                            }
                        }
                    }
                }
            }
            return 50 + cost; // 50 is the cost of performing lane change
        }

        int cost_current_lane() {
            double closest_vehicle_ahead_distance = ego_vehicle.closest_vehicle_ahead_distance(vehicle_map);
            if (closest_vehicle_ahead_distance < 100) {
                return (100 - closest_vehicle_ahead_distance);
            }
            return 0;
        }

        void process_cost_function() {
            int cost_lane_keeping = this->cost_current_lane();
            int cost_lane_change_left = this->cost_left_lane();
            int cost_lane_change_right = this->cost_right_lane();
            string message = std::to_string(cost_lane_change_left) + " <-- " + std::to_string(cost_lane_keeping) + " --> " + std::to_string(cost_lane_change_right);
            Utils::print_message(message);

            int LANE_CHANGE_MAX_COST = 80;
            bool lane_change_left_criteria_met = cost_lane_change_left <= cost_lane_keeping &&
                cost_lane_change_left <= cost_lane_change_right &&
                cost_lane_change_left <= LANE_CHANGE_MAX_COST;
            bool lane_change_right_criteria_met = cost_lane_change_right <= cost_lane_keeping &&
                cost_lane_change_right <= cost_lane_change_left &&
                cost_lane_change_right <= LANE_CHANGE_MAX_COST;

            if (lane_change_left_criteria_met) {
                Utils::print_message("<<<<<");
                ego_vehicle.prepare_lane_change_left();
            } else if (lane_change_right_criteria_met) {
                Utils::print_message(">>>>>");
                ego_vehicle.prepare_lane_change_right();
            } else {
                ego_vehicle.keep_lane(vehicle_map);
            }
        }
        /* Move the logics related to cost function and lane change to a separate file - end */

        json get_smooth_curve() {
            // this->ego_vehicle.print_vehicle_information();
            // Adding the leftover points from the previously generated path to the next path
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            for (int i = 0; i < this->get_previous_path_x().size(); i++) {
                next_x_vals.push_back(this->get_previous_path_x()[i]);
                next_y_vals.push_back(this->get_previous_path_y()[i]);
            }

            // Fetching 5 points (in car frame) to draw spline for the path
            this->process_cost_function();
            int lane = this->ego_vehicle.get_ref_lane();
            double gap = props.distance_between_waypoints_in_meters();

            tuple<vector<double>, vector<double>> five_route_coordinates_in_world_frame = this->get_5_points_for_spline_generation(gap, lane);
            vector<double> world_x= std::get<0>(five_route_coordinates_in_world_frame);
            vector<double> world_y = std::get<1>(five_route_coordinates_in_world_frame);
            
            tuple<vector<double>, vector<double>> five_route_coordinates_in_car_frame = this->get_coordinates_in_car_frame(five_route_coordinates_in_world_frame);
            vector<double> ptsx = std::get<0>(five_route_coordinates_in_car_frame);
            vector<double> ptsy = std::get<1>(five_route_coordinates_in_car_frame);

            // Generating the spline over the 5 points from above
            tk::spline s;
            s.set_points(ptsx, ptsy);

            // Adding points to the next path to compensate the points travelled since last cycle.
            double target_x = props.lane_switch_in_meters();
            double target_y = s(target_x);
            double target_dist = sqrt(target_x * target_x + target_y * target_y);

            int total_points_in_route = props.suggested_points_count();
            int remaining_points = this->get_previous_path_x().size();
            int points_to_add = total_points_in_route - remaining_points;

            double ref_vel = this->ego_vehicle.get_ref_speed();
            double distance_in_meters_between_points = props.refresh_rate_in_seconds() * ref_vel * MPH_TO_METERS_PER_SECOND;
            double N = target_dist / distance_in_meters_between_points;

            double x_add_on = 0.0;
            /*
            double ref_yaw = this->ego_vehicle.get_yaw();
            */
            /* Remote this code - start */
            double ref_x_temp = world_x[1];
            double ref_y_temp = world_y[1];

            double ref_x_prev_temp = world_x[0];
            double ref_y_prev_temp = world_y[0];

            double ref_yaw = atan2(ref_y_temp - ref_y_prev_temp, ref_x_temp - ref_x_prev_temp);
            /* Remote this code - end */

            for (int i = 1; i <= points_to_add; i++) {
                double x_point = x_add_on + distance_in_meters_between_points;
                double y_point = s(x_point);

                x_add_on = x_point;
                double x_ref = x_point;
                double y_ref = y_point;

                // rotate back to normal after rotating it earlier
                x_point = (x_ref * cos(ref_yaw)) - (y_ref * sin(ref_yaw));
                y_point = (x_ref * sin(ref_yaw)) + (y_ref * cos(ref_yaw));

                x_point += world_x[1];
                y_point += world_y[1];

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }

            json msgJson;
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

            return msgJson;
        }
};
#endif

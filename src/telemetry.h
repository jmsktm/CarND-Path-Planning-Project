#ifndef SDC_TELEMETRY_MODULE
#define SDC_TELEMETRY_MODULE
#include <tuple>
#include <string>
#include <vector>
#include <math.h> /* atan2 */

#include "helpers.h"
#include "props.h"
#include "map.h"

#include "json.hpp"
using nlohmann::json;

using std::string;
using std::tuple;
using std::vector;

class Telemetry {
    private:
        Map map;
        Props props;
        string str;
        json data;

    public:
        Telemetry() {}
        ~Telemetry() {}

        Telemetry(string str) {
            this->str = str;
            data = this->_getTelemetryData();
        }

        bool _hasEventMessageData() {
            // "42" at the start of the message means there's a websocket message event.
            // The 4 signifies a websocket message
            // The 2 signifies a websocket event
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

        double get_x() {
            return data["x"];
        }

        double get_y() {
            return data["y"];
        }

        double get_s() {
            return data["s"];
        }

        double get_d() {
            return data["d"];
        }

        double get_yaw() {
            return data["yaw"];
        }

        double get_speed() {
            return data["speed"];
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
            double x = get_x();
            double y = get_y();
            double yaw = get_yaw();

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
            double s = get_s();
            double target_d = props.get_s_by_lane(target_lane);

            vector<double> next_waypoint0 = getXY(s + 1 * gap, target_d, map.get_waypoints_s(), map.get_waypoints_x(), map.get_waypoints_y());
            vector<double> next_waypoint1 = getXY(s + 2 * gap, target_d, map.get_waypoints_s(), map.get_waypoints_x(), map.get_waypoints_y());
            vector<double> next_waypoint2 = getXY(s + 3 * gap, target_d, map.get_waypoints_s(), map.get_waypoints_x(), map.get_waypoints_y());

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
};
#endif

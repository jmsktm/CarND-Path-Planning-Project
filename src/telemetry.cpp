#include <string>

#include "../src/json.hpp"
using nlohmann::json;

using std::string;

class Telemetry {
    private:
        string str;
        json data;

    public:
        Telemetry(string str) {
            this->str = str;
            data = this->_getTelemetryData();
        }

        bool _hasEventMessageData() {
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
};
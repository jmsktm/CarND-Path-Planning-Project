#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "../src/json.hpp"
#include "../src/telemetry.h"
#include "../src/utils.h"

#include <map>
#include <tuple>

using std::map;
using std::tuple;

void check_equal(double num1, double num2, int num_digits) {
    double first = Utils::round(num1, num_digits);
    double second = Utils::round(num2, num_digits);

    REQUIRE(first == second);
}

TEST_CASE("telemetry.cpp >> _hasEventMessageData()") {
    REQUIRE_FALSE(Telemetry("")._hasEventMessageData());
    REQUIRE_FALSE(Telemetry("test")._hasEventMessageData());
    REQUIRE_FALSE(Telemetry("42[]")._hasEventMessageData());
    REQUIRE(Telemetry("42[\"telemetry\",{}]")._hasEventMessageData());
}

TEST_CASE("telemetry.cpp >> _getEventMessageData() with valid input data") {
    string result = Telemetry("42[\"telemetry\",{\"a\":\"b\"}]")._getEventMessageData();
    REQUIRE(result == "[\"telemetry\",{\"a\":\"b\"}]");
}

TEST_CASE("telemetry.cpp >> _hasTelemetryData()") {
    bool falseCase = Telemetry("42[\"test\",{\"a\":\"b\"}]")._hasTelemetryData();
    REQUIRE_FALSE(falseCase);

    bool trueCase = Telemetry("42[\"telemetry\",{\"a\":\"b\"}]")._hasTelemetryData();
    REQUIRE(trueCase);
}

TEST_CASE("telemetry.cpp >> _getTelemetryData()") {
    SECTION("Does not contain telemetry data") {
        json result = Telemetry("42[\"test\",{\"a\":\"b\"}]")._getTelemetryData();
        json expected = json({});
        REQUIRE(result == expected); 
    }
    SECTION("Contains telemetry data") {
        json result = Telemetry("42[\"telemetry\",{\"a\":\"b\"}]")._getTelemetryData();
        json expected = R"(
            {
                "a": "b"
            }
        )"_json;
        REQUIRE(result == expected);
    }
}

TEST_CASE("Get properties from the (valid) telemetry data") {
    json input = R"(
        {
            "x": 1.0,
            "y": 2.0,
            "s": 3.0,
            "d": 4.0,
            "yaw": 1.23,
            "speed": 6.0,
            "previous_path_x": [1.0, 2.0],
            "previous_path_y": [3.0, 4.0],
            "end_path_s": 7.0,
            "end_path_d": 8.0,
            "sensor_fusion": [
                [1, 0.1],
                [2, 0.2]
            ]
        }
    )"_json;
    string str = input.dump();
    Telemetry telemetry = Telemetry("42[\"telemetry\"," + str + "]");
    Vehicle ego_vehicle = telemetry.get_ego_vehicle();
    REQUIRE(ego_vehicle.get_x() == 1.0);
    REQUIRE(ego_vehicle.get_y() == 2.0);
    REQUIRE(ego_vehicle.get_s() == 3.0);
    REQUIRE(ego_vehicle.get_d() == 4.0);
    REQUIRE(ego_vehicle.get_yaw() == 1.23);
    REQUIRE(ego_vehicle.get_speed() == 6.0);
    REQUIRE(telemetry.get_previous_path_x() == R"([1.0, 2.0])"_json);
    REQUIRE(telemetry.get_previous_path_y() == R"([3.0, 4.0])"_json);
    REQUIRE(telemetry.get_end_path_s() == 7.0);
    REQUIRE(telemetry.get_end_path_d() == 8.0);
    REQUIRE(telemetry.get_sensor_fusion() == R"([[1, 0.1], [2, 0.2]])"_json);
}

TEST_CASE("Return last two points when previous path is not available") {
    json data = R"(
        {
            "x": 1.0,
            "y": 2.0,
            "s": 3.0,
            "d": 4.0,
            "yaw": 1.23,
            "speed": 6.0
        }
    )"_json;
    string data_string = "42[\"telemetry\"," + data.dump() + "]";
    Telemetry telemetry = Telemetry(data_string);
    Vehicle ego_vehicle = telemetry.get_ego_vehicle();
    REQUIRE(ego_vehicle.get_x() == 1.0);
    REQUIRE(ego_vehicle.get_y() == 2.0);
    REQUIRE(ego_vehicle.get_s() == 3.0);
    REQUIRE(ego_vehicle.get_d() == 4.0);
    REQUIRE(ego_vehicle.get_yaw() == 1.23);
    REQUIRE(ego_vehicle.get_speed() == 6.0);

    tuple<vector<double>, vector<double>> last_two_points = telemetry.derive_last_two_points_from_current_pos();
    vector<double> ptsx = std::get<0>(last_two_points);
    vector<double> ptsy = std::get<1>(last_two_points);

    // Rounding off to account for unpredictably changing precision
    #define roundz(x,d) ((floor(((x)*pow(10,d))+.5))/pow(10,d))

    vector<double> actual_ptsx = { roundz(ptsx[0], 3), roundz(ptsx[1], 3)};
    vector<double> actual_ptsy = { roundz(ptsy[0], 3), roundz(ptsy[1], 3)};

    vector<double> expected_ptsx = { roundz(0.666, 3), roundz(1.00, 3) };
    vector<double> expected_ptsy = { roundz(1.058, 3), roundz(2.00, 3) };

    REQUIRE(actual_ptsx == expected_ptsx);
    REQUIRE(actual_ptsy == expected_ptsy);
}

TEST_CASE("Get last two points as a tuple from previous_path_x and previous_path_y") {
    json data = R"(
        {
            "previous_path_x": [ 100.00, 200.00, 300.00 ],
            "previous_path_y": [ 120.00, 220.00, 320.00 ]
        }
    )"_json;
    string data_string = "42[\"telemetry\"," + data.dump() + "]";
    Telemetry telemetry = Telemetry(data_string);

    REQUIRE(telemetry.get_previous_path_x() == R"([ 100.00, 200.00, 300.00 ])"_json);
    REQUIRE(telemetry.get_previous_path_y() == R"([ 120.00, 220.00, 320.00 ])"_json);

    vector<double> expected_ptsx = { 200.00, 300.00 };
    vector<double> expected_ptsy = { 220.00, 320.00 };

    tuple<vector<double>, vector<double>> last_two_points = telemetry.get_last_two_points_from_previous_path();
    vector<double> actual_ptsx = std::get<0>(last_two_points);
    vector<double> actual_ptsy = std::get<1>(last_two_points);

    REQUIRE(expected_ptsx == actual_ptsx);
    REQUIRE(expected_ptsy == actual_ptsy);
}

TEST_CASE("Get the next 3 evenly spaced waypoints as tuples of x and y coordinates") {
    // Rounding off to account for unpredictably changing precision
    #define roundz(x,d) ((floor(((x)*pow(10,d))+.5))/pow(10,d))

    json data = R"(
        {
            "s": 100.0
        }
    )"_json;
    string data_string = "42[\"telemetry\"," + data.dump() + "]";
    Telemetry telemetry = Telemetry(data_string);
    Vehicle ego_vehicle = telemetry.get_ego_vehicle();
    REQUIRE(ego_vehicle.get_s() == 100.0);

    tuple<vector<double>, vector<double>> last_3_waypoints = telemetry.get_next_3_evenly_spaced_waypoints(30, 2);
    vector<double> ptsx = std::get<0>(last_3_waypoints);
    vector<double> ptsy = std::get<1>(last_3_waypoints);

    vector<double> actual_ptsx = { roundz(ptsx[0], 3), roundz(ptsx[1], 3), roundz(ptsx[2], 3) };
    vector<double> actual_ptsy = { roundz(ptsy[0], 3), roundz(ptsy[1], 3), roundz(ptsy[2], 3) };

    // The below numbers were not calculated manually, but rather taken
    // directly from the the function invocation, as it's based on the
    // route on the map, and not easily calculable.
    vector<double> expected_ptsx = { roundz(914.6791548971, 3), roundz(945.623244928, 3), roundz(976.4068813369, 3) };
    vector<double> expected_ptsy = { roundz(1124.88, 3), roundz(1126.162, 3), roundz(1130.728, 3) };

    REQUIRE(actual_ptsx == expected_ptsx);
    REQUIRE(actual_ptsy == expected_ptsy);
}

TEST_CASE("Get the 5 points for spline generation when less than 2 previous path points exist") {
    json data = R"(
        {
            "x": 100.0,
            "y": 200.0,
            "yaw": 30.0,
            "speed": 123.0,
            "s": 100.0,
            "d": 4.0,
            "previous_path_x": [],
            "previous_path_y": []
        }
    )"_json;
    string data_string = "42[\"telemetry\"," + data.dump() + "]";
    Telemetry telemetry = Telemetry(data_string);

    double gap = 30.0;
    double target_lane = 2;
    tuple<vector<double>, vector<double>> five_points = telemetry.get_5_points_for_spline_generation(gap, target_lane);
    vector<double> ptsx = std::get<0>(five_points);
    vector<double> ptsy = std::get<1>(five_points);

    int precision = 4;
    vector<double> actual_ptsx = { roundz(ptsx[0], precision), roundz(ptsx[1], precision), roundz(ptsx[2], precision), roundz(ptsx[3], precision), roundz(ptsx[4], precision) };
    vector<double> actual_ptsy = { roundz(ptsy[0], precision), roundz(ptsy[1], precision), roundz(ptsy[2], precision), roundz(ptsy[3], precision), roundz(ptsy[4], precision) };

    vector<double> expected_ptsx = { roundz(99.8457, precision), roundz(100.00, precision), roundz(914.6791548971, precision), roundz(945.623244928, precision), roundz(976.4068813369, precision) };
    vector<double> expected_ptsy = { roundz(200.988, precision), roundz(200.00, precision), roundz(1124.8797, precision), roundz(1126.1618, precision), roundz(1130.7282, precision) };

    REQUIRE(actual_ptsx == expected_ptsx);
    REQUIRE(actual_ptsy == expected_ptsy);
}

TEST_CASE("Get the 5 points for spline generation when more than 2 previous path points exist") {
    json data = R"(
        {
            "s": 100.0,
            "previous_path_x": [ 100.00, 200.00, 300.00 ],
            "previous_path_y": [ 120.00, 220.00, 320.00 ]
        }
    )"_json;
    string data_string = "42[\"telemetry\"," + data.dump() + "]";
    Telemetry telemetry = Telemetry(data_string);

    double gap = 30.0;
    double target_lane = 2;
    tuple<vector<double>, vector<double>> five_points = telemetry.get_5_points_for_spline_generation(gap, target_lane);
    vector<double> ptsx = std::get<0>(five_points);
    vector<double> ptsy = std::get<1>(five_points);

    vector<double> actual_ptsx = { roundz(ptsx[0], 3), roundz(ptsx[1], 3), roundz(ptsx[2], 3), roundz(ptsx[3], 3), roundz(ptsx[4], 3) };
    vector<double> actual_ptsy = { roundz(ptsy[0], 3), roundz(ptsy[1], 3), roundz(ptsy[2], 3), roundz(ptsy[3], 3), roundz(ptsy[4], 3) };

    vector<double> expected_ptsx = { roundz(200, 3), roundz(300.00, 3), roundz(914.6791548971, 3), roundz(945.623244928, 3), roundz(976.4068813369, 3) };
    vector<double> expected_ptsy = { roundz(220, 3), roundz(320.00, 3), roundz(1124.88, 3), roundz(1126.162, 3), roundz(1130.728, 3) };

    REQUIRE(actual_ptsx == expected_ptsx);
    REQUIRE(actual_ptsy == expected_ptsy);
}

TEST_CASE("Convert from world frame coordinates to car frame coordinates") {
    // Rounding off to account for unpredictably changing precision
    #define roundz(x,d) ((floor(((x)*pow(10,d))+.5))/pow(10,d))

    vector<double> world_frame_x = { roundz(200, 3), roundz(300.00, 3), roundz(914.6791548971, 3), roundz(945.623244928, 3), roundz(976.4068813369, 3) };
    vector<double> world_frame_y = { roundz(220, 3), roundz(320.00, 3), roundz(1124.88, 3), roundz(1126.162, 3), roundz(1130.728, 3) };
    tuple<vector<double>, vector<double>> world_frame_coordinates = std::make_tuple(world_frame_x, world_frame_y);

    Telemetry telemetry = Telemetry("42[\"telemetry\",{}]");
    tuple<vector<double>, vector<double>> car_frame_coordinates = telemetry.get_coordinates_in_car_frame(world_frame_coordinates);
    vector<double> car_frame_coordinates_x = std::get<0>(car_frame_coordinates);
    vector<double> car_frame_coordinates_y = std::get<1>(car_frame_coordinates);

    vector<double> actual_x = {
        roundz(car_frame_coordinates_x[0], 3),
        roundz(car_frame_coordinates_x[1], 3),
        roundz(car_frame_coordinates_x[2], 3),
        roundz(car_frame_coordinates_x[3], 3),
        roundz(car_frame_coordinates_x[4], 3)
    };

    vector<double> actual_y = {
        roundz(car_frame_coordinates_y[0], 3),
        roundz(car_frame_coordinates_y[1], 3),
        roundz(car_frame_coordinates_y[2], 3),
        roundz(car_frame_coordinates_y[3], 3),
        roundz(car_frame_coordinates_y[4], 3)
    };

    vector<double> expected_x = { roundz(-141.4213562373, 3), roundz(0.0, 3), roundz(1003.7797951944, 3), roundz(1026.5670183249, 3), roundz(1051.5632430399, 3) };
    vector<double> expected_y = { roundz(0.0, 3), roundz(0.0, 3), roundz(134.492, 3), roundz(113.518, 3), roundz(94.979, 3) };

    REQUIRE(actual_x == expected_x);
    REQUIRE(actual_y == expected_y);
}

TEST_CASE("get_ego_vehicle_data()") {
    json telemetry_data = R"(
        {
            "x": 1410.48,
            "y": 1128.67,
            "yaw": 30,
            "speed": 55.92341,
            "s": 642.3477,
            "d": 52.12466
        }
    )"_json;

    string data_string = "42[\"telemetry\"," + telemetry_data.dump() + "]";
    Telemetry telemetry = Telemetry(data_string);
    vector<double> actual = telemetry.get_ego_vehicle_data();
    vector<double> expected = { -1, 1410.48, 1128.67, 8.626267, -55.254097, 642.3477, 52.12466 };
    
    double num_digits = 4; // 4 digits after decimal
    bool result = Utils::vectors_equal(actual, expected, num_digits);
    REQUIRE(result == true);
}

TEST_CASE("ego vehicle should be set on instantiation of Telemetry") {
    json telemetry_data = R"(
        {
            "x": 1410.48,
            "y": 1128.67,
            "yaw": 30,
            "speed": 55.92341,
            "s": 642.3477,
            "d": 52.12466
        }
    )"_json;

    string data_string = "42[\"telemetry\"," + telemetry_data.dump() + "]";
    Telemetry telemetry = Telemetry(data_string);
    Vehicle vehicle = telemetry.get_ego_vehicle();

    int num_digits = 4;
    check_equal(vehicle.get_id(), -1, num_digits);
    check_equal(vehicle.get_x(), 1410.48, num_digits);
    check_equal(vehicle.get_y(), 1128.67, num_digits);
    check_equal(vehicle.get_vx(), 8.626267, num_digits);
    check_equal(vehicle.get_vy(), -55.254097, num_digits);
    check_equal(vehicle.get_s(), 642.3477, num_digits);
    check_equal(vehicle.get_d(), 52.12466, num_digits);
    check_equal(vehicle.get_speed(), 55.923409, num_digits);
    check_equal(vehicle.get_yaw(), -1.415926, num_digits);
}

TEST_CASE("Telemetry should include neighboring vehicles in a map") {
    json input = R"(
        {
            "x": 1.0,
            "y": 2.0,
            "s": 3.0,
            "d": 4.0,
            "yaw": 1.23,
            "speed": 6.0,
            "previous_path_x": [1.0, 2.0],
            "previous_path_y": [3.0, 4.0],
            "end_path_s": 7.0,
            "end_path_d": 8.0,
            "sensor_fusion": [
                [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                [1, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6]
            ]
        }
    )"_json;
    string str = input.dump();
    Telemetry telemetry = Telemetry("42[\"telemetry\"," + str + "]");
    map<int, Vehicle> vehicles = telemetry.get_surrounding_vehicles();
    REQUIRE(vehicles.size() == 2);
    REQUIRE(vehicles.find(0) != vehicles.end());
    REQUIRE(vehicles.find(1) != vehicles.end());

    int precision = 4;
    Vehicle vehicle0 = vehicles[0];
    check_equal(vehicle0.get_x(), 0.1, precision);
    check_equal(vehicle0.get_y(), 0.2, precision);
    check_equal(vehicle0.get_vx(), 0.3, precision);
    check_equal(vehicle0.get_vy(), 0.4, precision);
    check_equal(vehicle0.get_s(), 0.5, precision);
    check_equal(vehicle0.get_d(), 0.6, precision);
    check_equal(vehicle0.get_speed(), 0.5, precision);
    check_equal(vehicle0.get_yaw(), 0.927295, precision);

    Vehicle vehicle1 = vehicles[1];
    check_equal(vehicle1.get_x(), 1.1, precision);
    check_equal(vehicle1.get_y(), 1.2, precision);
    check_equal(vehicle1.get_vx(), 1.3, precision);
    check_equal(vehicle1.get_vy(), 1.4, precision);
    check_equal(vehicle1.get_s(), 1.5, precision);
    check_equal(vehicle1.get_d(), 1.6, precision);
    check_equal(vehicle1.get_speed(), 1.910497, precision);
    check_equal(vehicle1.get_yaw(), 0.822418, precision);
}
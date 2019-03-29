#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "../src/json.hpp"
#include "../src/telemetry.h"

#include <tuple>

using std::tuple;

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
            "yaw": 5.0,
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
    REQUIRE(telemetry.get_x() == 1.0);
    REQUIRE(telemetry.get_y() == 2.0);
    REQUIRE(telemetry.get_s() == 3.0);
    REQUIRE(telemetry.get_d() == 4.0);
    REQUIRE(telemetry.get_yaw() == 5.0);
    REQUIRE(telemetry.get_speed() == 6.0);
    REQUIRE(telemetry.get_previous_path_x() == R"([1.0, 2.0])"_json);
    REQUIRE(telemetry.get_previous_path_y() == R"([3.0, 4.0])"_json);
    REQUIRE(telemetry.get_end_path_s() == 7.0);
    REQUIRE(telemetry.get_end_path_d() == 8.0);
    REQUIRE(telemetry.get_sensor_fusion() == R"([[1, 0.1], [2, 0.2]])"_json);
}

TEST_CASE("Return last two points when only current point and yaw are known") {
    json data = R"(
        {
            "x": 100.0,
            "y": 200.0,
            "yaw": 45.0
        }
    )"_json;
    string data_string = "42[\"telemetry\"," + data.dump() + "]";
    Telemetry telemetry = Telemetry(data_string);
    REQUIRE(telemetry.get_x() == 100.0);
    REQUIRE(telemetry.get_y() == 200.0);
    REQUIRE(telemetry.get_yaw() == 45.0);

    tuple<vector<double>, vector<double>> last_two_points = telemetry.derive_last_two_points_from_current_pos();
    vector<double> ptsx = std::get<0>(last_two_points);
    vector<double> ptsy = std::get<1>(last_two_points);

    // Rounding off to account for unpredictably changing precision
    #define roundz(x,d) ((floor(((x)*pow(10,d))+.5))/pow(10,d))

    vector<double> actual_ptsx = { roundz(ptsx[0], 3), roundz(ptsx[1], 3)};
    vector<double> actual_ptsy = { roundz(ptsy[0], 3), roundz(ptsy[1], 3)};

    vector<double> expected_ptsx = { roundz(99.4746780112, 3), roundz(100.00, 3) };
    vector<double> expected_ptsy = { roundz(199.149096475, 3), roundz(200.00, 3) };

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
    REQUIRE(telemetry.get_s() == 100.0);

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
            "yaw": 45.0,
            "s": 100.0,
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

    vector<double> actual_ptsx = { roundz(ptsx[0], 3), roundz(ptsx[1], 3), roundz(ptsx[2], 3), roundz(ptsx[3], 3), roundz(ptsx[4], 3) };
    vector<double> actual_ptsy = { roundz(ptsy[0], 3), roundz(ptsy[1], 3), roundz(ptsy[2], 3), roundz(ptsy[3], 3), roundz(ptsy[4], 3) };

    vector<double> expected_ptsx = { roundz(99.4746780112, 3), roundz(100.00, 3), roundz(914.6791548971, 3), roundz(945.623244928, 3), roundz(976.4068813369, 3) };
    vector<double> expected_ptsy = { roundz(199.149096475, 3), roundz(200.00, 3), roundz(1124.88, 3), roundz(1126.162, 3), roundz(1130.728, 3) };

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
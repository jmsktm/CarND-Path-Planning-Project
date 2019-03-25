#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "../src/json.hpp"
#include "../src/telemetry.cpp"

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
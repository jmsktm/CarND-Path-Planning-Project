#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "../src/props.h"

TEST_CASE("Read properties from config") {
    Props props = Props();
    REQUIRE(props.speed_limit() == 49.5);
    REQUIRE(props.current_road_lanes() == 3);
    REQUIRE(props.opposite_road_lanes() == 3);
    REQUIRE(props.lane_width_in_meters() == 4);
    REQUIRE(props.cycle_period_in_ms() == 20);
    REQUIRE(props.suggested_points_count() == 50);
    REQUIRE(props.start_lane() == 1);
}

SCENARIO("props / get_s_by_lane()") {
    Props props = Props();
    double width = props.lane_width_in_meters();

    auto [lane, s] = GENERATE(table<int, double> ({
        { 0, 2.0 },
        { 1, 6.0 },
        { 2, 10.0 }
    }));

    GIVEN("Given the width of the lane is " << width << " meters")
    WHEN("The car is in lane " << lane)
    THEN("The value for s should equal " << s << " meters") {
        REQUIRE(props.get_s_by_lane(lane) == s);
    }
}
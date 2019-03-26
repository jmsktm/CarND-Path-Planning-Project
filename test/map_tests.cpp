#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <string>
#include <vector>

#include "../src/map.h"
#include "../src/waypoint.h"

using std::string;
using std::vector;

TEST_CASE("Map should contain a list of endpoints on initialization") {
    Map map = Map();
    vector<Waypoint> waypoints = map.getWaypoints();
    REQUIRE(waypoints.size() == 181); // There are 181 waypoints in provided data file.

    REQUIRE(map.get_waypoints_x().size() == 181);
    REQUIRE(map.get_waypoints_y().size() == 181);
    REQUIRE(map.get_waypoints_s().size() == 181);
    REQUIRE(map.get_waypoints_dx().size() == 181);
    REQUIRE(map.get_waypoints_dy().size() == 181);
}

SCENARIO("Test for waypoints against data from provided file") {
    Map map = Map();
    vector<Waypoint> waypoints = map.getWaypoints();

    // Rounding off to account for unpredictably changing precision
    #define roundz(x,d) ((floor(((x)*pow(10,d))+.5))/pow(10,d))

    auto [row, x, y, s, dx, dy] = GENERATE(table<int, double, double, double, double, double> ({
        { 0, 784.6, 1135.57, 0, -0.0235983, -0.999722},
        { 1, 815.268, 1134.93, 30.6745, -0.0109948, -0.99994 }
    }));

    GIVEN("Given the records in data/highway_map.csv")
    WHEN("I consider row: " << row)
    THEN("I should have (x, y, s, dx, dy) == (" << x << ", " << y << ", " << s << ", " << dx << ", " << dy << ")") {
        std::cout << "(" << x << ", " << y << ", " << s << ", " << dx << ", " << dy << ")" << std::endl;
        REQUIRE(roundz(waypoints.at(row).get_x(), 2) == roundz(x, 2));
        REQUIRE(roundz(waypoints.at(row).get_y(), 2) == roundz(y, 2));
        REQUIRE(roundz(waypoints.at(row).get_s(), 2) == roundz(s, 2));
        REQUIRE(roundz(waypoints.at(row).get_dx(), 2) == roundz(dx, 2));
        REQUIRE(roundz(waypoints.at(row).get_dy(), 2) == roundz(dy, 2));
    }
}
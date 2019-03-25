#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "../src/config.cpp"

#include <string>

TEST_CASE("Read project config (string values; first level) from config/config.json") {
    ConfigInterface *config = new Config();
    string key = "license";
    string expected = "MIT";
    string result = config->value("license");
    
    REQUIRE(result == expected);
}

SCENARIO("Read road configs (int values; second level) from config/config.json") {
    ConfigInterface *config = new Config();
    auto [key1, key2, expected] = GENERATE(table<string, string, int> ({
        { "road", "speed-limit", 50 },
        { "road", "total-lanes", 6 },
        { "road", "lane-width-in-meters", 4 }
    }));

    GIVEN("We have road configs in config/config.json")
    WHEN("The keys are: " << key1 << " and " << key2)
    THEN("The value should be " << expected) {
        int result = config->intValue(key1, key2);
        REQUIRE(result == expected);
    }
}

SCENARIO("Read road configs (string values; second level) from config/config.json") {
    ConfigInterface *config = new Config();
    auto [key1, key2, expected] = GENERATE(table<string, string, string> ({
        { "project", "name", "CarND-Path-Planning-Project" },
        { "project", "author", "James Singh" }
    }));

    GIVEN("We have project config in config/config.json")
    WHEN("The keys are: " << key1 << " and " << key2)
    THEN("The value should be " << expected) {
        string result = config->value(key1, key2);
        REQUIRE(result == expected);
    }
}

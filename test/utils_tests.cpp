#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include <vector>
#include "../src/utils.h"

using std::vector;

TEST_CASE("vectors_equal(): different size") {
    vector<double> vector1 = {1.0, 2.0};
    vector<double> vector2 = {1.0, 2.0, 3.0};
    bool result = Utils::vectors_equal(vector1, vector2, 4);
    REQUIRE (result == false);
}

TEST_CASE("vectors_equal(): expect equal") {
    vector<double> vector1 = {1.00010, 1.00010, 1.00010, 1.00010};
    vector<double> vector2 = {1.00011, 1.00012, 1.00013, 1.00014};
    bool result = Utils::vectors_equal(vector1, vector2, 4);
    REQUIRE (result == true);
}

TEST_CASE("vectors_equal(): expect not equal") {
    vector<double> vector1 = {1.00010, 1.00010, 1.00010, 1.00010, 1.00010};
    vector<double> vector2 = {1.00011, 1.00012, 1.00013, 1.00014, 1.00015};
    bool result = Utils::vectors_equal(vector1, vector2, 4);
    REQUIRE (result == false);
}

TEST_CASE("round(double x, int d) should round x to d places after the decimal") {
    double result1 = Utils::round(123.44444444, 4);
    REQUIRE (result1 == 123.4444);

    double result2 = Utils::round(123.55555555, 4);
    REQUIRE (result2 == 123.5556);
}

TEST_CASE("Test for numbers_equal at the boundary") {
    bool result1 = Utils::numbers_equal(123.44444444, 123.44443333, 4);
    REQUIRE(result1 == true);

    bool result2 = Utils::numbers_equal(123.44444444, 123.44445555, 4);
    REQUIRE(result2 == false);
}
#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include <string>
#include <vector>
#include "../src/testme.cpp"

using std::string;
using std::vector;
using namespace Catch::Matchers;

TEST_CASE("Unit test for greet") {
    string result = greet("James");
    REQUIRE(result == "Hello James");

    REQUIRE_THAT("james", Contains("me"));
    CHECK_THAT("james", Contains("me"));

    CHECK_THAT("james", StartsWith("jam"));
    CHECK_THAT("james", EndsWith("mes"));

    CHECK_FALSE("James" == "james");
    CHECK_FALSE(false);
}

TEST_CASE("Matchers can be composed of both && and ||") {
    CHECK_THAT("think different", (Contains("be") || Contains("different")) && Contains("think"));
}

TEST_CASE("Using sections") {
    SECTION( "Contains (element) ") {
        vector<int> v {1, 2};
        CHECK_THAT(v, VectorContains( 1 ));
        CHECK_THAT(v, VectorContains( 2 ));
    }
}

static auto eatCucumbers (int start, int eat ) -> int { return start - eat; }
SCENARIO("test") {
    auto [start, eat, left] = GENERATE(table<int, int, int> ({
        { 12, 5, 7 },
        { 20, 5, 15 }
    }));

    GIVEN("There are " << start << " cucumbers")
    WHEN("I eat " << eat << " cucumbers")
    THEN("I should have " << left << " cucumbers") {
        REQUIRE(eatCucumbers(start, eat) == left);
    }
}

static auto square( int i ) -> int { return i*i; };

TEST_CASE( "Random numbers in range", "[.][approvals]") {
    auto x = GENERATE ( random (-10000, 10000 ) );
    CAPTURE( x );
    REQUIRE ( square(x) == 0 );
}
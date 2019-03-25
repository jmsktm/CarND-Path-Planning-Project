#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "../src/spline.h"
#include <string>
#include <vector>

using std::string;
using std::vector;
using tk::spline;

TEST_CASE("Test for spline") {
    vector<double> X {1, 3, 4, 5};
    vector<double> Y {1, 3, 4, 5};

    spline s;
    s.set_points(X, Y);

    double Y2 = s(2.0);
    REQUIRE(Y2 == 2.0);
}
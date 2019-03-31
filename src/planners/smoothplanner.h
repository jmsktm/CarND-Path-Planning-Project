#ifndef SDC_SMOOTH_PLANNER_MODULE
#define SDC_SMOOTH_PLANNER_MODULE

#include "../spline.h"
#include "../map.h"
#include "../telemetry.h"
#include "../helpers.h"

/**
 * Smooth planner keeps the lane in a smooth trajectory in lane#2 (middle lane).
 */
class SmoothPlanner {
    private:
        Map map;
        Telemetry& telemetry;
    public:
        ~SmoothPlanner() {}
        SmoothPlanner(Map &map, Telemetry &t): telemetry(t) {
            this->map = map;
        }
    public:
        /**
         * Define a path made up of (x,y) points that the car will visit
         * sequentially every .02 seconds
         */
        json getRoute() {
            return this->telemetry.get_smooth_curve();
        }
};
#endif
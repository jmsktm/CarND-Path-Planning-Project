#ifndef SDC_SIMPLE_PLANNER_MODULE
#define SDC_SIMPLE_PLANNER_MODULE

#include "../map.h"
#include "../telemetry.h"
#include "../helpers.h"

/**
 * Simple planner keeps the lane along lane#2 (middle lane) in straight lines
 * parallel to the waypoints.
 */
class SimplePlanner {
    private:
        Map map;
        Telemetry telemetry;
    public:
        SimplePlanner() {}
        ~SimplePlanner() {}
        SimplePlanner(Map &map, Telemetry &telemetry) {
            this->map = map;
            this->telemetry = telemetry;
        }
    public:
        /**
         * Define a path made up of (x,y) points that the car will visit
         * sequentially every .02 seconds
         */
        json getRoute() {
            json msgJson;
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            double s = this->telemetry.get_s();

            double dist_inc = 0.5;
            double next_d = 6;
            for (int i = 0; i < 50; ++i) {
                double next_s = s + (i+1) * dist_inc;
                vector<double> xy = getXY(next_s, next_d, this->map.get_waypoints_s(), this->map.get_waypoints_x(), this->map.get_waypoints_y());
                next_x_vals.push_back(xy[0]);
                next_y_vals.push_back(xy[1]);
            }
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

            return msgJson;
        }
};
#endif
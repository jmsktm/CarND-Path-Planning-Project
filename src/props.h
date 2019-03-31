#ifndef SDC_PROPS_MODULE
#define SDC_PROPS_MODULE

#include "config.cpp"

class Props {
    private:
        Config config;
    public:
        Props() {
            this->config = Config();
        };
        ~Props() {};

        double speed_limit() {
            return config.doubleValue("road", "speed-limit");
        }

        double current_road_lanes() {
            return config.doubleValue("road", "current-road-lanes");
        }

        double opposite_road_lanes() {
            return config.doubleValue("road", "opposite-road-lanes");
        }

        double lane_width_in_meters() {
            return config.doubleValue("road", "lane-width-in-meters");
        }

        double cycle_period_in_ms() {
            return config.doubleValue("setup", "cycle-period-in-ms");
        }

        double suggested_points_count() {
            return config.doubleValue("setup", "suggested-points-count");
        }

        int start_lane() {
            return config.intValue("setup", "start-lane");
        }

        double get_d_by_lane(int lane) {
            return (lane + 0.5) * lane_width_in_meters();
        }

        double lane_switch_in_meters() {
            return config.doubleValue("setup", "lane-switch-in-meters");
        }

        double refresh_rate_in_ms() {
            return config.doubleValue("setup", "refresh-rate-in-ms");
        }

        double refresh_rate_in_seconds() {
            return this->refresh_rate_in_ms() / 1000;
        }

        double distance_between_waypoints_in_meters() {
            return config.doubleValue("road", "distance-between-waypoints");
        }
};
#endif
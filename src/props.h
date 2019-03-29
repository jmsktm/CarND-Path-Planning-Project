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

        double get_s_by_lane(int lane) {
            return (lane + 0.5) * lane_width_in_meters();
        }
};
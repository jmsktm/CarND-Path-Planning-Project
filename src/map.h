#ifndef SDC_MAP_MODULE
#define SDC_MAP_MODULE

#include "Eigen-3.3/Eigen/QR"

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include "waypoint.h"

using std::string;
using std::vector;

class Map {
    private:
        string path = "../data/highway_map.csv";
        double max_s = 6945.554;
        vector<Waypoint> waypoints;

        vector<double> map_waypoints_x;
        vector<double> map_waypoints_y;
        vector<double> map_waypoints_s;
        vector<double> map_waypoints_dx;
        vector<double> map_waypoints_dy;
    public:
        Map() {
            std::ifstream in_map_(path.c_str(), std::ifstream::in);

            string line;
            while (getline(in_map_, line)) {
                std::istringstream iss(line);
                double x;
                double y;
                float s;
                float dx;
                float dy;
                iss >> x;
                iss >> y;
                iss >> s;
                iss >> dx;
                iss >> dy;

                map_waypoints_x.push_back(x);
                map_waypoints_y.push_back(y);
                map_waypoints_s.push_back(s);
                map_waypoints_dx.push_back(dx);
                map_waypoints_dy.push_back(dy);

                Waypoint waypoint = Waypoint(x, y, s, dx, dy);
                this->waypoints.push_back(waypoint);
            }
        }

        vector<Waypoint> getWaypoints() {
            return waypoints;
        }

        vector<double> get_waypoints_x() {
            return map_waypoints_x;
        }

        vector<double> get_waypoints_y() {
            return map_waypoints_y;
        }

        vector<double> get_waypoints_s() {
            return map_waypoints_s;
        }

        vector<double> get_waypoints_dx() {
            return map_waypoints_dx;
        }

        vector<double> get_waypoints_dy() {
            return map_waypoints_dy;
        }
};
#endif
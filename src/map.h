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

                Waypoint waypoint = Waypoint(x, y, s, dx, dy);
                this->waypoints.push_back(waypoint);
            }
        }

        vector<Waypoint> getWaypoints() {
            return waypoints;
        }
};
#endif
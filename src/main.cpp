#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "telemetry.h"
#include "waypoint.h"
#include "map.h"

using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;
  Map map;

  h.onMessage([&map]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    string str(data);
    Telemetry telemetry = Telemetry(str);

    if (telemetry._hasTelemetryData()) {
      double x = telemetry.get_x();
      double y = telemetry.get_y();
      double s = telemetry.get_s();
      double d = telemetry.get_d();
      double yaw = telemetry.get_yaw();
      double speed = telemetry.get_speed();
      auto previous_path_x = telemetry.get_previous_path_x();
      auto previous_path_y = telemetry.get_previous_path_y();
      double end_path_s = telemetry.get_end_path_s();
      double end_path_d = telemetry.get_end_path_d();
      auto sensor_fusion = telemetry.get_sensor_fusion();

      json msgJson;
      vector<double> next_x_vals;
      vector<double> next_y_vals;

      /**
       * TODO: define a path made up of (x,y) points that the car will visit
       * sequentially every .02 seconds
       */
      double dist_inc = 0.4;
      double next_d = 6;
      for (int i = 0; i < 50; ++i) {
        double next_s = s + (i+1) * dist_inc;
        vector<double> xy = getXY(next_s, next_d, map.get_waypoints_s(), map.get_waypoints_x(), map.get_waypoints_y());
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
      }
      msgJson["next_x"] = next_x_vals;
      msgJson["next_y"] = next_y_vals;

      auto msg = "42[\"control\","+ msgJson.dump()+"]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    } else {
      // Manual driving
      std::string msg = "42[\"manual\",{}]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
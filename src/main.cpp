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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
      double dist_inc = 0.5;
      for (int i = 0; i < 50; ++i) {
        next_x_vals.push_back(x+(dist_inc*i)*cos(deg2rad(yaw)));
        next_y_vals.push_back(y+(dist_inc*i)*sin(deg2rad(yaw)));
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
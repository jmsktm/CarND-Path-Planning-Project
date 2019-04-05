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
#include "utils.h"
#include "planners/smoothplanner.h"
//#include "planners/simpleplanner.h"

using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;
  Map map;
  Vehicle ego_vehicle;

  h.onMessage([&map, &ego_vehicle]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    string str(data);
    Telemetry telemetry = Telemetry(str, ego_vehicle);
    SmoothPlanner planner = SmoothPlanner(map, telemetry);

    if (telemetry._hasTelemetryData()) {
      json msgJson = planner.getRoute();
      auto msg = "42[\"control\","+ msgJson.dump()+"]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    } else {
      std::string msg = "42[\"manual\",{}]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
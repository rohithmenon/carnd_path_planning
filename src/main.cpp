#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "BehaviourPlanner.h"
#include "Localization.h"
#include "Network.h"
#include "Prediction.h"
#include "TrajectoryPlanner.h"
#include "Utils.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

void HandleData(vector<double>& map_waypoints_x,
                vector<double>& map_waypoints_y,
                vector<double>& map_waypoints_s,
                vector<double>& map_waypoints_dx,
                vector<double>& map_waypoints_dy,
                std::shared_ptr<common::Network> network,
                trajectory_planner::TrajectoryPlanner& trajectory_planner,
                utils::CoordConversions& coord_conversions,
                string s,
                uWS::WebSocket<uWS::SERVER>& ws) {
  if (s != "") {
    auto j = json::parse(s);

    string event = j[0].get<string>();

    if (event == "telemetry") {

      // Main car's localization Data
      double car_x = j[1]["x"];
      double car_y = j[1]["y"];
      double car_s = j[1]["s"];
      double car_d = j[1]["d"];
      double car_yaw = j[1]["yaw"];
      double car_speed = j[1]["speed"];

      // Previous path data given to the Planner
      auto previous_path_x = j[1]["previous_path_x"];
      auto previous_path_y = j[1]["previous_path_y"];
      // Previous path's end s and d values
      double end_path_s = j[1]["end_path_s"];
      double end_path_d = j[1]["end_path_d"];

      // Sensor Fusion Data, a list of all other cars on the same side of the road.
      auto sensor_fusion = j[1]["sensor_fusion"];

      msg::Localization localization;
      localization.d_m = car_d;
      localization.s_m = car_s;
      localization.velocity_mph = car_speed;
      auto xy = coord_conversions.getXY(car_s, car_d);
      localization.x_m = xy[0];
      localization.y_m = xy[1];
      localization.yaw_rad = utils::deg2rad(car_yaw);
      localization.millis = common::Clock::NowMillis();
      network->Publish("localization", localization);

      msg::SensorFusion sensor_fusion_msg;
      for (auto per_vehicle : sensor_fusion) {
        msg::VehicleState& vehicle_state = sensor_fusion_msg.vehicle_states[per_vehicle[0]];
        vehicle_state.s_m = per_vehicle[5];
        vehicle_state.d_m = per_vehicle[6];
        double vel_x = per_vehicle[3];
        double vel_y = per_vehicle[4];
        vehicle_state.velocity_mps = std::sqrt(vel_x * vel_x + vel_y * vel_y);
        vehicle_state.acceleration_mps2 = 0;
      }
      network->Publish("sensor_fusion", sensor_fusion_msg);

      json msgJson;

      vector<double> next_x_vals;
      vector<double> next_y_vals;
      std::vector<msg::WayPointXY> previous_trajectory;
      for (int i = 0; i < previous_path_x.size(); ++i) {
        msg::WayPointXY waypoint;
        waypoint.x_m = previous_path_x[i];
        waypoint.y_m = previous_path_y[i];
        previous_trajectory.emplace_back(waypoint);
      }
      std::vector<msg::WayPointXY> trajectory = trajectory_planner.GetTrajectory(previous_trajectory);
      for (auto way_pt : trajectory) {
        next_x_vals.emplace_back(way_pt.x_m);
        next_y_vals.emplace_back(way_pt.y_m);
      }

      // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
      msgJson["next_x"] = next_x_vals;
      msgJson["next_y"] = next_y_vals;

      auto msg = "42[\"control\","+ msgJson.dump()+"]";

      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

    }
  } else {
    // Manual driving
    std::string msg = "42[\"manual\",{}]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  }

}

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  utils::CoordConversions coord_conversions(map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
  std::shared_ptr<common::Network> network(new common::Network());
  behaviour_planner::BehaviourPlanner behaviour_planner(network, 1, 1, 48.0, 30, 15, 3, 2);
  prediction::Prediction prediction(network);
  trajectory_planner::TrajectoryPlanner trajectory_planner(network, coord_conversions, 48.0, 1, 1, 50, 0.02);
  localization::Localization localization(network);

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, network, &
								trajectory_planner, &coord_conversions](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);
      HandleData(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy, network, trajectory_planner, coord_conversions, s, ws);
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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

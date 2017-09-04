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
#include "Helper.h"
#include "Vehicle.h"
#include "constants.h"
#include "Road.h"

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {

  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<float> map_waypoints_s;
  vector<float> map_waypoints_dx;
  vector<float> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  Helper helper;

  vector<double> SIGMA_S;
  SIGMA_S.push_back(SIGMA_S1);
  SIGMA_S.push_back(SIGMA_S_DOT);
  SIGMA_S.push_back(SIGMA_S_DDOT);// s, s_dot, s_double_dot
  vector<double> SIGMA_D;
  SIGMA_D.push_back(SIGMA_D1);
  SIGMA_D.push_back(SIGMA_D_DOT);
  SIGMA_D.push_back(SIGMA_D_DDOT);

  vector<string> cost_list_;
  vector<double> cost_weight_;

  cost_list_.push_back("time_diff");
  cost_weight_.push_back(TIME_DIFF_COST_WEIGHT);

  cost_list_.push_back("s_diff");
  cost_weight_.push_back(TRAJ_DIFF_COST_WEIGHT);

  cost_list_.push_back("d_diff");
  cost_weight_.push_back(TRAJ_DIFF_COST_WEIGHT);

  cost_list_.push_back("efficiency");
  cost_weight_.push_back(EFFICIENCY_COST_WEIGHT);

  cost_list_.push_back("max_jerk");
  cost_weight_.push_back(MAX_JERK_COST_WEIGHT);

  cost_list_.push_back("total_jerk");
  cost_weight_.push_back(AVG_JERK_COST_WEIGHT);

  cost_list_.push_back("collision");
  cost_weight_.push_back(COLLISION_COST_WEIGHT);

  cost_list_.push_back("buffer");
  cost_weight_.push_back(BUFFER_COST_WEIGHT);

  cost_list_.push_back("max_acc");
  cost_weight_.push_back(MAX_ACCEL_COST_WEIGHT);

  cost_list_.push_back("total_acc");
  cost_weight_.push_back(AVG_ACCEL_COST_WEIGHT);

  cost_list_.push_back("speed_limit");
  cost_weight_.push_back(SPEED_LIMIT_COST_WEIGHT);

  Vehicle ego_vehicle(cost_weight_,
                      cost_list_,
                      SIGMA_S,
                      SIGMA_D,
                      SIGMA_T,
                      VEHICLE_RADIUS,
                      EXPECTED_ACC_IN_ONE_SEC,
                      MAX_INSTANTANEOUS_ACCEL,
                      EXPECTED_JERK_IN_ONE_SEC,
                      MAX_INSTANTANEOUS_JERK,
                      helper.mph2mps(SPEED_LIMIT),
                      N_SAMPLES_PATH,
                      DESIRED_TIME_HEADWAY,
                      CAR_FOLLOW_MIN_DISTANCE,
                      MAX_CAR_FOLLOW_TIME_HEADWAY,
                      TRACK_LENGTH,
                      PRINT_COST);
  ego_vehicle.set_planning_horizon(PREDICTION_HORIZON);

  Road road(FREE_DISTANCE, DELTA_SPEED);

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,
                      &ego_vehicle,&helper,&road](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = helper.hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"]; // center: 2 left 6 middle 10 right
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          vector<double> vehicle_state;
          vehicle_state.push_back(car_x);
          vehicle_state.push_back(car_y);
          vehicle_state.push_back(car_s);
          vehicle_state.push_back(car_d);
          vehicle_state.push_back(car_yaw);
          vehicle_state.push_back(car_speed);

          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          int subpath_size = min(PREVIOUS_PATH_POINTS_TO_KEEP, (int)previous_path_x.size());



          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
          vector<Vehicle_Prediction> other_vehicle_list;
          for(int i = 0;i<sensor_fusion.size();i++){
            Vehicle_Prediction other_vehicle(sensor_fusion[i],"xy");
            other_vehicle_list.push_back(other_vehicle);
          }

          road.update_road(other_vehicle_list,previous_path_x,PATH_DT,ego_vehicle);

          double planning_horizon = NUM_PATH_POINTS * PATH_DT - subpath_size * PATH_DT;
          ego_vehicle.set_initial_state(vehicle_state,previous_path_x,previous_path_y);
          ego_vehicle.update_state(road);
          double path_sample_dt = PATH_GENERATION_DT;
          if(previous_path_x.size()>2){
            path_sample_dt+=0.4;
          }
          vector<vector<double>> traj_xy = ego_vehicle.get_XY_trajectory(
                                            previous_path_x,
                                            previous_path_y,
                                            NUM_PATH_POINTS,
                                            PATH_DT,
                                            path_sample_dt,
                                            map_waypoints_s,
                                            map_waypoints_x,
                                            map_waypoints_y
                                    );

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          next_x_vals = traj_xy[0];
          next_y_vals = traj_xy[1];

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
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

















































































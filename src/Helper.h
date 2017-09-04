//
// Created by xnhuang on 8/8/17.
//

#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
//#include "gnuplot-iostream.h"
#include "spline.h"

#ifndef PATH_PLANNING_HELPER_H
#define PATH_PLANNING_HELPER_H

using namespace std;

using namespace std;

struct trajectory{
  vector<double> coefficients_s;
  vector<double> coefficients_d;
  double time;
  double cost;
  vector<double> goal_s;
  vector<double> goal_d;
  vector<double> cost_list;
};

class Vehicle_Prediction{
  int lane;
  vector<double> start_frenet;
public:

  Vehicle_Prediction(vector<double> start_, string type);

  ~Vehicle_Prediction();

  vector<double> predict(double t);

  void set_initial(double t);

  int get_lane();

  double get_s();

  double get_speed();
};

class Helper {
public:
  Helper();
  virtual ~Helper();
  // For converting back and forth between radians and degrees.
  double pi();

  double deg2rad(double x);

  double rad2deg(double x);

  double mph2mps(double mph);

  string hasData(string s);

  double distance(double x1, double y1, double x2, double y2);

  int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

  int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

  // Transform from Frenet s,d coordinates to Cartesian x,y
  vector<double> getXY(double s, double d, vector<float>& maps_s, vector<double>& maps_x, vector<double>& maps_y);

  vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y,
                                            double interval, int output_size);
  double logistic(double x);

  double polyval(vector<double>& coefficients, double x);

  vector<double> polyval_multi(vector<vector<double>>& coefficients_collection, double x);

  vector<double> differentiate(vector<double> coefficients);

  double nearest_approach_to_any_vehicle(trajectory& candidate, vector<Vehicle_Prediction>& vehicle_list);

  double nearest_approach(trajectory& candidate_traj, Vehicle_Prediction& candidate_vehicle);

  vector<vector<double>> get_N_derivatives(vector<double> coefficients, int N = 3);

//  void show_trajectory(vector<double> s_coeffs, vector<double> d_coeffs, double time_desire);

//  void show_trajectory(vector<double> s_coeffs, vector<double> d_coeffs, double time_desire, Vehicle_Prediction vehicle);
};
#endif //PATH_PLANNING_HELPER_H

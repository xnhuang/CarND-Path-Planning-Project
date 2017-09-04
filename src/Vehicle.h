//
// Created by xnhuang on 8/11/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <string>
#include "Path.h"
#include "Helper.h"
#include "spline.h"
#include "Road.h"

using namespace std;
class Road;

class Vehicle : public Path {

  vector<double> prev_s;
  vector<double> prev_d;
  vector<double> state_raw;
  int print_cost;

  double track_length;
  double desired_time_headway;
  double max_car_follow_time_headway;
  double car_follow_min_distance;

  double planning_horizon;
  double delay_time;
  int car_lane_initial;
  int target_lane;
  string car_status;

  trajectory car_trajectory;

  int goal_lane = 1;

  void d2lane();

public:
  Vehicle();

  Vehicle(vector<double>& cost_weight_,
          vector<string>& cost_list_,
          vector<double>& sigma_s_,
          vector<double>& sigma_d_,
          double sigma_t_,
          double vehicle_radius_,
          double expect_avg_acc_,
          double expect_max_acc_,
          double expect_avg_jerk_,
          double expect_max_jerk_,
          double speed_limit_,
          int goal_sample_size_,
          double desired_time_headway_,
          double car_follow_min_distance_,
          double max_car_follow_time_headway_,
          double track_length_,
          int print_cost_
  );

  ~Vehicle();

  Vehicle(const Vehicle& other);

  void set_initial_state(const vector<double> &state_raw_,
                         vector<double>& previous_path_x,
                         vector<double>& previous_path_y);

  void update_state(Road& road);

  trajectory realize_action(Road& road, int target_lane);

  trajectory get_trajectory();

  int d2lane(double d);

  vector<vector<double>> get_XY_trajectory(
          vector<double>& previous_path_x,
          vector<double>& previous_path_y,
          int path_point_size,
          double path_dt,
          double corse_path_dt,
          vector<float>& maps_s,
          vector<double>& maps_x,
          vector<double>& maps_y
  );

//  calculate lane center given lane number
  double lane2d(int lane);

  void set_planning_horizon(double planning_horizon_);

  vector<double> get_prev_s();
  vector<double> get_prev_d();
  int get_prev_lane();

};


#endif //PATH_PLANNING_VEHICLE_H

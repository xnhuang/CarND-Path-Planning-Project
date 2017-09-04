//
// Created by xnhuang on 8/8/17.
//

#ifndef TRAJECTORYGENERATION_PATH_H
#define TRAJECTORYGENERATION_PATH_H
#include <iostream>
#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <random>
#include <cassert>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Helper.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class Path {
  vector<double> cost_weight;
  vector<string> cost_list;

  vector<double> sigma_s;
  vector<double> sigma_d;
  double sigma_t;

  double vehicle_radius;

  double expect_avg_acc;
  double expect_max_acc;
  double expect_avg_jerk;
  double expect_max_jerk;
  double speed_limit;

  int goal_sample_size;

  double calculate_cost(
          trajectory candidate,
          vector<double>& start_s,
          vector<double>& start_d,
          vector<double>& goal_s,
          vector<double>& goal_d,
          double desired_time,
          vector<Vehicle_Prediction>& predictions,
          string cost_type,
          int print_output = 0
  );


  vector<double> JMT(
          vector<double>& start,
          vector<double>& end,
          double T
  );
  double time_diff_cost(trajectory candidate,
                        vector<double>& start_s,
                        vector<double>& start_d,
                        vector<double>& goal_s,
                        vector<double>& goal_d,
                        double desired_time,
                        vector<Vehicle_Prediction>& predictions
  );
  double s_diff_cost(trajectory candidate,
                     vector<double>& start_s,
                     vector<double>& start_d,
                     vector<double>& goal_s,
                     vector<double>& goal_d,
                        double desired_time,
                        vector<Vehicle_Prediction>& predictions
  );
  double d_diff_cost(trajectory candidate,
                     vector<double>& start_s,
                     vector<double>& start_d,
                     vector<double>& goal_s,
                     vector<double>& goal_d,
                        double desired_time,
                        vector<Vehicle_Prediction>& predictions
  );
  double efficiency_cost(trajectory candidate,
                         vector<double>& start_s,
                         vector<double>& start_d,
                         vector<double>& goal_s,
                         vector<double>& goal_d,
                        double desired_time,
                        vector<Vehicle_Prediction>& predictions
  );
  double max_jerk_cost(trajectory candidate,
                       vector<double>& start_s,
                       vector<double>& start_d,
                       vector<double>& goal_s,
                       vector<double>& goal_d,
                        double desired_time,
                        vector<Vehicle_Prediction>& predictions
  );
  double total_jerk_cost(trajectory candidate,
                         vector<double>& start_s,
                         vector<double>& start_d,
                         vector<double>& goal_s,
                         vector<double>& goal_d,
                        double desired_time,
                        vector<Vehicle_Prediction>& predictions
  );
  double collision_cost(trajectory candidate,
                        vector<double>& start_s,
                        vector<double>& start_d,
                        vector<double>& goal_s,
                        vector<double>& goal_d,
                        double desired_time,
                        vector<Vehicle_Prediction>& predictions
  );
  double buffer_cost(trajectory candidate,
                     vector<double>& start_s,
                     vector<double>& start_d,
                     vector<double>& goal_s,
                     vector<double>& goal_d,
                        double desired_time,
                        vector<Vehicle_Prediction>& predictions
  );
  double max_accel_cost(trajectory candidate,
                        vector<double>& start_s,
                        vector<double>& start_d,
                        vector<double>& goal_s,
                        vector<double>& goal_d,
                        double desired_time,
                        vector<Vehicle_Prediction>& predictions
  );
  double total_accel_cost(trajectory candidate,
                          vector<double>& start_s,
                          vector<double>& start_d,
                          vector<double>& goal_s,
                          vector<double>& goal_d,
                        double desired_time,
                        vector<Vehicle_Prediction>& predictions
  );
  double exceeds_speed_limit_cost(trajectory candidate,
                                  vector<double>& start_s,
                                  vector<double>& start_d,
                                  vector<double>& goal_s,
                                  vector<double>& goal_d,
                          double desired_time,
                          vector<Vehicle_Prediction>& predictions
  );

public:
  Helper helper;

  Path();

  Path(vector<double> cost_weight_, vector<string> cost_list_);

  Path(vector<double> cost_weight_,
       vector<string> cost_list_,
       vector<double> sigma_s_,
       vector<double> sigma_d_,
       double sigma_t_,
       double vehicle_radius_,
       double expect_avg_acc_,
       double expect_max_acc_,
       double expect_avg_jerk_,
       double expect_max_jerk_,
       double speed_limit_,
       int goal_sample_size_);

  Path(const Path& other);

  Path& operator =(const Path& other);

  virtual ~Path();

  void set_sigma(vector<double> sigma_s_,vector<double> sigma_d_,double sigma_t_);

  void set_vehicle_radius(double vehicle_radius_);

  void set_expect_avg_acc(double expect_avg_acc_);

  void set_expect_max_acc(double expect_max_acc_);

  void set_expect_avg_jerk(double expect_avg_jerk_);

  void set_expect_max_jerk(double expect_max_jerk_);

  void set_speed_limit(double speed_limit_);

  void set_goal_sample_size(int goal_sample_size_);

  double get_speed_limit();
  double get_vehicle_radius();
  trajectory get_best_trajectory(
          vector<double>& start_s,
          vector<double>& start_d,
          vector<double>& goal_s,
          vector<double>& goal_d,
          double desired_time,
          vector<Vehicle_Prediction>& predictions,
          int print_cost = 0
  );

};


#endif //TRAJECTORYGENERATION_PATH_H

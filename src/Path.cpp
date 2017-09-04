//
// Created by xnhuang on 8/8/17.
//

#include "Path.h"

Path::Path() = default;

Path::~Path() = default;

Path::Path(vector<double> cost_weight_, vector<string> cost_list_){
  assert(cost_list_.size()==cost_weight_.size());
  cost_weight = cost_weight_;
  cost_list = cost_list_;
};

void Path::set_sigma(vector<double> sigma_s_, vector<double> sigma_d_, double sigma_t_) {
  sigma_d = sigma_d_;
  sigma_s = sigma_s_;
  sigma_t = sigma_t_;
}

Path::Path(vector<double> cost_weight_, vector<string> cost_list_, vector<double> sigma_s_,
                vector<double> sigma_d_, double sigma_t_, double vehicle_radius_, double expect_avg_acc_,
                double expect_max_acc_, double expect_avg_jerk_, double expect_max_jerk_, double speed_limit_,
                int goal_sample_size_) {
  assert(cost_list_.size()==cost_weight_.size());
  cost_weight = cost_weight_;
  cost_list = cost_list_;
  sigma_d = sigma_d_;
  sigma_s = sigma_s_;
  sigma_t = sigma_t_;
  vehicle_radius = vehicle_radius_;
  expect_avg_acc = expect_avg_acc_;
  expect_max_acc = expect_max_acc_;
  expect_avg_jerk = expect_avg_jerk_;
  expect_max_jerk = expect_max_jerk_;
  speed_limit = speed_limit_;
  goal_sample_size = goal_sample_size_;
}

Path::Path(const Path& other) {
  cost_weight = other.cost_weight;
  cost_list = other.cost_list;
  sigma_d = other.sigma_d;
  sigma_s = other.sigma_s;
  sigma_t = other.sigma_t;
  vehicle_radius = other.vehicle_radius;
  expect_avg_acc = other.expect_avg_acc;
  expect_max_acc = other.expect_max_acc;
  expect_avg_jerk = other.expect_avg_jerk;
  expect_max_jerk = other.expect_max_jerk;
  speed_limit = other.speed_limit;
  goal_sample_size = other.goal_sample_size;
}

Path& Path::operator=(const Path& other) {
  cost_weight = other.cost_weight;
  cost_list = other.cost_list;
  sigma_d = other.sigma_d;
  sigma_s = other.sigma_s;
  sigma_t = other.sigma_t;
  vehicle_radius = other.vehicle_radius;
  expect_avg_acc = other.expect_avg_acc;
  expect_max_acc = other.expect_max_acc;
  expect_avg_jerk = other.expect_avg_jerk;
  expect_max_jerk = other.expect_max_jerk;
  speed_limit = other.speed_limit;
  goal_sample_size = other.goal_sample_size;
  return *this;
}

void Path::set_vehicle_radius(double vehicle_radius_) {
  vehicle_radius = vehicle_radius_;
}

void Path::set_expect_avg_acc(double expect_avg_acc_) {
  expect_avg_acc = expect_avg_acc_;
}

void Path::set_expect_max_acc(double expect_max_acc_) {
  expect_max_acc = expect_max_acc_;
}

void Path::set_expect_avg_jerk(double expect_avg_jerk_){
  expect_avg_jerk = expect_avg_jerk_;
}

void Path::set_expect_max_jerk(double expect_max_jerk_){
  expect_max_jerk = expect_max_jerk_;
}

void Path::set_speed_limit(double speed_limit_){
  speed_limit = speed_limit_;
}

void Path::set_goal_sample_size(int goal_sample_size_){
  goal_sample_size = goal_sample_size_;
};

double Path::get_speed_limit() {
  return speed_limit;
}

double Path::get_vehicle_radius() {
  return vehicle_radius;
}

trajectory Path::get_best_trajectory(vector<double> &start_s,
                                     vector<double> &start_d,
                                     vector<double> &goal_s,
                                     vector<double> &goal_d,
                                     double desired_time,
                                     vector<Vehicle_Prediction> &predictions,
                                     int print_cost) {


    vector<double> s_coefficients = this->JMT(start_s, goal_s, desired_time);
    vector<double> d_coefficients = this->JMT(start_d, goal_d, desired_time);
    trajectory candidate;
    candidate.time = desired_time;
    candidate.coefficients_s = s_coefficients;
    candidate.coefficients_d = d_coefficients;
//    calculate total cost of trajectory
    double total_cost = 0;
    for(int cost_id = 0;cost_id<cost_list.size();cost_id++){
      double cost_individual = calculate_cost(candidate,start_s,start_d,goal_s,goal_d, desired_time,
                                              predictions, cost_list[cost_id], print_cost);
      total_cost += cost_weight[cost_id]*cost_individual;
      candidate.cost_list.push_back(cost_individual);
    }
    if(print_cost==1){
      cout << "total cost : " << total_cost << endl;
    }
  candidate.cost = total_cost;
  return candidate;
}

double Path::calculate_cost(
        trajectory candidate,
        vector<double>& start_s,
        vector<double>& start_d,
        vector<double>& goal_s,
        vector<double>& goal_d,
        double desired_time,
        vector<Vehicle_Prediction>& predictions,
        string cost_type,
        int print_output
) {
  double cost = 0.0;
  if(cost_type.compare("time_diff")==0){
    cost = time_diff_cost(candidate, start_s,start_d,goal_s,goal_d, desired_time, predictions);
  }
  else if(cost_type.compare("s_diff")==0){
    cost =  s_diff_cost(candidate, start_s,start_d,goal_s,goal_d, desired_time, predictions);
  }
  else if(cost_type.compare("d_diff")==0){
    cost = d_diff_cost(candidate, start_s,start_d,goal_s,goal_d, desired_time, predictions);
  }
  else if(cost_type.compare("efficiency")==0) {
    cost = efficiency_cost(candidate, start_s,start_d,goal_s,goal_d, desired_time, predictions);
  }
  else if(cost_type.compare("max_jerk")==0){
    cost =  max_jerk_cost(candidate, start_s,start_d,goal_s,goal_d, desired_time, predictions);
  }
  else if(cost_type.compare("total_jerk")==0){
    cost = total_jerk_cost(candidate, start_s,start_d,goal_s,goal_d, desired_time, predictions);
  }
  else if(cost_type.compare("collision")==0){
    cost = collision_cost(candidate, start_s,start_d,goal_s,goal_d, desired_time, predictions);
  }
  else if(cost_type.compare("buffer")==0){
    cost = buffer_cost(candidate, start_s,start_d,goal_s,goal_d, desired_time, predictions);
  }
  else if(cost_type.compare("max_acc")==0){
    cost = max_accel_cost(candidate, start_s,start_d,goal_s,goal_d, desired_time, predictions);
  }
  else if(cost_type.compare("total_acc")==0){
    cost = total_accel_cost(candidate, start_s,start_d,goal_s,goal_d, desired_time, predictions);
  }
  else if(cost_type.compare("speed_limit")==0){
    cost = exceeds_speed_limit_cost(candidate, start_s,start_d,goal_s,goal_d, desired_time, predictions);
  }
  if(print_output==1){
    cout << "cost type: " << cost_type << "; value = " << cost << endl;
  }
  return cost;
};

double Path::time_diff_cost(trajectory candidate,
                            vector<double>& start_s,
                            vector<double>& start_d,
                            vector<double>& goal_s,
                            vector<double>& goal_d,
                            double desired_time,
                            vector<Vehicle_Prediction>& predictions
){
  return helper.logistic(float(abs(candidate.time-desired_time)) / desired_time);
};

double Path::s_diff_cost(trajectory candidate,
                         vector<double>& start_s,
                         vector<double>& start_d,
                         vector<double>& goal_s,
                         vector<double>& goal_d,
                         double desired_time,
                         vector<Vehicle_Prediction>& predictions
){
  vector<double> s = candidate.coefficients_s;
  double T = candidate.time;

  vector<vector<double>> traj_diff_coeff = helper.get_N_derivatives(s,2);
  vector<double> traj_terminal = helper.polyval_multi(traj_diff_coeff,T);

  double cost = 0.0;
  for(int dim_id = 0;dim_id<sigma_s.size();dim_id++){
    double diff = abs(traj_terminal[dim_id]-goal_s[dim_id]);
    cost += helper.logistic(diff/sigma_s[dim_id]);
  };
  return cost;
};

double Path::d_diff_cost(trajectory candidate,
                         vector<double>& start_s,
                         vector<double>& start_d,
                         vector<double>& goal_s,
                         vector<double>& goal_d,
                         double desired_time,
                         vector<Vehicle_Prediction>& predictions
){
  vector<double> d = candidate.coefficients_d;
  double T = candidate.time;

  vector<vector<double>> traj_diff_coeff = helper.get_N_derivatives(d,2);
  vector<double> traj_terminal = helper.polyval_multi(traj_diff_coeff,T);

  double cost = 0.0;
  for(int dim_id = 0;dim_id<sigma_d.size();dim_id++){
    double diff = abs(traj_terminal[dim_id]-goal_d[dim_id]);
    cost += helper.logistic(diff/sigma_d[dim_id]);
  };
  return cost;
};

double Path::efficiency_cost(trajectory candidate,
                             vector<double>& start_s,
                             vector<double>& start_d,
                             vector<double>& goal_s,
                             vector<double>& goal_d,
                             double desired_time,
                             vector<Vehicle_Prediction>& predictions
){
  vector<double> s = candidate.coefficients_s;
  double T = candidate.time;

  double avg_spd = helper.polyval(s,T)/T;
  double target_s = goal_s[0];
  double target_spd = target_s/T;

  return helper.logistic(2*float(target_spd - avg_spd) / (avg_spd+0.1));
};

double Path::max_jerk_cost(trajectory candidate,
                           vector<double>& start_s,
                           vector<double>& start_d,
                           vector<double>& goal_s,
                           vector<double>& goal_d,
                           double desired_time,
                           vector<Vehicle_Prediction>& predictions
){
  vector<double> s = candidate.coefficients_s;
  vector<double> d = candidate.coefficients_d;

  vector<double> s_dot = helper.differentiate(s);
  vector<double> s_d_dot = helper.differentiate(s_dot);
  vector<double> jerk = helper.differentiate(s_d_dot);

  double max_jerk = 0.0;
  double dt = desired_time/100.0;

  for(int i = 0;i<100;i++){
    double t = dt*i;
    double jerk_t = helper.polyval(jerk,t);
    if(abs(jerk_t) > max_jerk){
      max_jerk = abs(jerk_t);
    };
  };

  if(max_jerk>expect_max_jerk){
    return 1.0;
  }

  return 0.0;
};


double Path::total_jerk_cost(trajectory candidate,
                             vector<double>& start_s,
                             vector<double>& start_d,
                             vector<double>& goal_s,
                             vector<double>& goal_d,
                             double desired_time,
                             vector<Vehicle_Prediction>& predictions
){
  vector<double> s = candidate.coefficients_s;
  vector<double> d = candidate.coefficients_d;

  vector<double> s_dot = helper.differentiate(s);
  vector<double> s_d_dot = helper.differentiate(s_dot);
  vector<double> jerk = helper.differentiate(s_d_dot);

  double total_jerk = 0.0;
  double dt = desired_time/100.0;

  for(int i = 0;i<100;i++){
    double t = dt*i;
    double jerk_t = helper.polyval(jerk,t);
    total_jerk += abs(jerk_t)*dt;
  };

  double avg_jerk = total_jerk/desired_time;

  return helper.logistic(avg_jerk / expect_avg_jerk );
};


double Path::collision_cost(trajectory candidate,
                            vector<double>& start_s,
                            vector<double>& start_d,
                            vector<double>& goal_s,
                            vector<double>& goal_d,
                            double desired_time,
                            vector<Vehicle_Prediction>& predictions
){
  double nearest = helper.nearest_approach_to_any_vehicle(candidate, predictions);
  if(nearest < 2*vehicle_radius){
    return 1.0;
  }
  return 0.0;
};


double Path::buffer_cost(trajectory candidate,
                         vector<double>& start_s,
                         vector<double>& start_d,
                         vector<double>& goal_s,
                         vector<double>& goal_d,
                         double desired_time,
                         vector<Vehicle_Prediction>& predictions
){
  double nearest = helper.nearest_approach_to_any_vehicle(candidate, predictions);
  return helper.logistic(2*vehicle_radius / nearest);
};


double Path::total_accel_cost(trajectory candidate,
                              vector<double>& start_s,
                              vector<double>& start_d,
                              vector<double>& goal_s,
                              vector<double>& goal_d,
                              double desired_time,
                              vector<Vehicle_Prediction>& predictions
){
  vector<double> s = candidate.coefficients_s;
  vector<double> d = candidate.coefficients_d;

  vector<double> s_dot = helper.differentiate(s);
  vector<double> s_d_dot = helper.differentiate(s_dot);

  double total_acc = 0.0;
  double dt = desired_time/100.0;

  for(int i = 0;i<100;i++){
    double t = dt*i;
    double acc = helper.polyval(s_d_dot,t);
    total_acc += abs(acc)*dt;
  };

  double avg_acc = total_acc/desired_time;

  return helper.logistic(avg_acc / expect_avg_acc );
};


double Path::max_accel_cost(trajectory candidate,
                            vector<double>& start_s,
                            vector<double>& start_d,
                            vector<double>& goal_s,
                            vector<double>& goal_d,
                            double desired_time,
                            vector<Vehicle_Prediction>& predictions
){
  vector<double> s = candidate.coefficients_s;
  vector<double> d = candidate.coefficients_d;

  vector<double> s_dot = helper.differentiate(s);
  vector<double> s_d_dot = helper.differentiate(s_dot);

  double max_acc = 0.0;
  double dt = desired_time/100.0;

  for(int i = 0;i<100;i++){
    double t = dt*i;
    double acc = helper.polyval(s_d_dot,t);
    if(abs(acc) > max_acc){
      max_acc = abs(acc);
    };
  };

  if(max_acc>expect_max_acc){
    return 1.0;
  }

  return 0.0;
};


double Path::exceeds_speed_limit_cost(trajectory candidate,
                                      vector<double>& start_s,
                                      vector<double>& start_d,
                                      vector<double>& goal_s,
                                      vector<double>& goal_d,
                                      double desired_time,
                                      vector<Vehicle_Prediction>& predictions
){
  vector<double> s = candidate.coefficients_s;
  vector<double> d = candidate.coefficients_d;

  vector<double> s_dot = helper.differentiate(s);

  double max_spd = 0.0;
  double dt = desired_time/100.0;

  for(int i = 0;i<100;i++){
    double t = dt*i;
    double spd = helper.polyval(s_dot,t);
    if(spd > max_spd){
      max_spd = spd;
    };
  };

  if(max_spd>speed_limit){
    return 1.0;
  }

  return 0.0;
};


vector<double> Path::JMT(vector<double> &start, vector<double> &end, double T) {
  MatrixXd kinetic(3,3);
  kinetic << pow(T,3),pow(T,4),pow(T,5),
          3*pow(T,2),4*pow(T,3),5*pow(T,4),
          6*pow(T,1),12*pow(T,2),20*pow(T,3);
  MatrixXd B(3,1);
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
          end[1]-(start[1]+start[2]*T),
          end[2]-start[2];
  Eigen::MatrixXd Ai = kinetic.inverse();
  Eigen::MatrixXd C = Ai*B;

  return {start[0], start[1], .5*start[2], C.data()[0], C.data()[1], C.data()[2]};
}
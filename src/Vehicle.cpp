//
// Created by xnhuang on 8/11/17.
//

#include "Vehicle.h"

Vehicle::Vehicle(){
  prev_s = {0,0,0};
  prev_d = {0,0,0};
  car_trajectory.cost = -1;
};

Vehicle::Vehicle(vector<double>& cost_weight_,
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
):
        Path(cost_weight_,cost_list_,sigma_s_,sigma_d_, sigma_t_,
             vehicle_radius_, expect_avg_acc_,expect_max_acc_,
             expect_avg_jerk_, expect_max_jerk_, speed_limit_,
             goal_sample_size_){

  desired_time_headway = desired_time_headway_;
  car_follow_min_distance = car_follow_min_distance_;
  max_car_follow_time_headway = max_car_follow_time_headway_;
  track_length = track_length_;
  print_cost = print_cost_;
  car_status = "KL";
  prev_s = {0,0,0};
  prev_d = {0,0,0};
  car_trajectory.cost = -1;
}

void Vehicle::set_initial_state(const vector<double> &state_raw_,
                                vector<double>& previous_path_x,
                                vector<double>& previous_path_y) {
  this->state_raw = state_raw_;
  state_raw[5] = helper.mph2mps(state_raw_[5]);
  this->d2lane();
  delay_time = previous_path_x.size()*0.02;
  if(car_trajectory.cost == -1){
    prev_s[0] = state_raw_[2];
    prev_d[0] = state_raw_[3];
    prev_s[1] = state_raw_[5];
    target_lane = car_lane_initial;
  }
}

Vehicle::Vehicle(const Vehicle& other):Path(other){
  prev_s = other.prev_s;
  prev_d = other.prev_d;
  d2lane();
  desired_time_headway = other.desired_time_headway;
  car_follow_min_distance = other.car_follow_min_distance;
  max_car_follow_time_headway = other.max_car_follow_time_headway;
  track_length = other.track_length;
  print_cost = other.print_cost;
  car_status = "KL";
  planning_horizon = other.planning_horizon;
}

Vehicle::~Vehicle() = default;

void Vehicle::d2lane() {
  car_lane_initial = int(floor((state_raw[3])/4));
}

int Vehicle::d2lane(double d){
  return int(floor((d)/4));
}

double Vehicle::lane2d(int lane) {
  return 2+4*lane;
}

vector<double> Vehicle::get_prev_s(){
  return prev_s;
};

vector<double> Vehicle::get_prev_d(){
  return prev_d;
};

int Vehicle::get_prev_lane() {
  return this->d2lane(prev_d[0]);
}

void Vehicle::update_state(Road& road) {
  double lane_spd = road.get_lane_spd(target_lane);
  double desired_distance = this->get_speed_limit()*desired_time_headway;
  cout << "lane speed = " << road.get_lane_spd(0) <<" "<< road.get_lane_spd(1)<<" "<<road.get_lane_spd(2)<< endl;
  cout << "lane available = " << road.free_lane(*this,0) <<" "<< road.free_lane(*this,1)<<" "<<road.free_lane(*this,2)<< endl;
  cout << "lane min_distance front = " << road.get_lane_min_distance(0)[0]
       <<" "<< road.get_lane_min_distance(1)[0]<<" "<<road.get_lane_min_distance(2)[0]<< endl;
  cout << "lane min_distance back = " << road.get_lane_min_distance(0)[1]
       <<" "<< road.get_lane_min_distance(1)[1]<<" "<<road.get_lane_min_distance(2)[1]<< endl;
  int available_lane;
  string next_car_status = "KL";
  if(car_status.compare("KL")==0){
    if(road.busy_lane(*this,target_lane,desired_distance)){
      cout << "current lane busy" << endl;
      available_lane = road.lane_change_available(*this);
      if(target_lane == available_lane){
        next_car_status = "KL";
      }
      if(target_lane > available_lane){
        next_car_status = "ILCL";
        cout << "LCL" << endl;
      }
      if(target_lane < available_lane){
        next_car_status = "ILCR";
        cout << "LCR" << endl;
      }
      target_lane = available_lane;
    }
  }
  if(car_status.compare("ILCL")==0){
    next_car_status = "LCL";
  }
  if(car_status.compare("ILCL")==0){
    next_car_status = "LCR";
  }
  if(this->car_status.compare("LCR")==0){
    if(abs(prev_d[0]-this->lane2d(target_lane))<0.2){
      next_car_status = "KL";
    }
    else{
      next_car_status = "LCR";
    }
  }
  if(this->car_status.compare("LCL")==0){
    if(abs(prev_d[0]-this->lane2d(target_lane))<0.2){
      next_car_status = "KL";
    }
    else{
      next_car_status = "LCL";
    }
  }
  trajectory best_trajectory = this->realize_action(road,target_lane);
  this->car_status = next_car_status;
  this->car_trajectory = best_trajectory;
}


trajectory Vehicle::realize_action(Road& road, int target_lane) {
//  simple logic
  vector<Vehicle_Prediction> other_vehicle_predict = road.get_lane_vehicle(target_lane);
  int target_vehicle_id = road.get_lane_min_distance_id(target_lane)[0];
  double min_distance = road.get_lane_min_distance(target_lane)[0];

  bool close_to_lead = false;
  bool car_following = false;

  double goal_lane_loc = this->lane2d(target_lane);
  vector<double> target_vehicle_state;
  if(target_vehicle_id != -1){
    target_vehicle_state = other_vehicle_predict[target_vehicle_id].predict(this->planning_horizon);
    double desired_distance;

    desired_distance = target_vehicle_state[1]*desired_time_headway
                       + car_follow_min_distance
                       + 2*this->get_vehicle_radius();
    double max_distance;

    max_distance = target_vehicle_state[1]*max_car_follow_time_headway
                       + car_follow_min_distance
                       + 2*this->get_vehicle_radius();
    close_to_lead = desired_distance>min_distance;
    car_following = max_distance>min_distance;
  }

  bool speed_low;
  double ref_spd;
  double speed_change = 0;
  int sign_speed_change;
//  switch between car following and free flow
  if(!car_following){
    ref_spd = (this->get_speed_limit());
//    cout << "free flow, ref_spd = "<< ref_spd<< endl;
    speed_low = prev_s[1]<ref_spd;
    if(speed_low){
//      cout << "accelerate" << endl;
      sign_speed_change = 1;
      speed_change = ref_spd - prev_s[1];
    }else{
//      cout << "keep" << endl;
    }
  }
  else{
    ref_spd = target_vehicle_state[1];
//    cout << "car following, ref_spd = "<< ref_spd << endl;
    if(close_to_lead){
//      cout << "decelerate" << endl;
      sign_speed_change = -1;
      speed_change = prev_s[1];
    }else{
      speed_low = prev_s[1]<ref_spd;
      if(speed_low){
//        cout << "accelerate" << endl;
        sign_speed_change = 1;
        speed_change = ref_spd - prev_s[1];
        speed_change = min(speed_change, this->get_speed_limit() - prev_s[1]);
      }else{
//        cout << "decelerate" << endl;
        sign_speed_change = -1;
        speed_change = prev_s[1] - ref_spd;
      }
    }
  }
  vector<double> goal_d = {goal_lane_loc,0,0};
  double desired_speed = prev_s[1] + sign_speed_change*speed_change;
  vector<double> goal_s = {
          (desired_speed + prev_s[1]) / 2 * this->planning_horizon + prev_s[0],
          desired_speed,
          (desired_speed - prev_s[1]) / this->planning_horizon};

  trajectory free_flow = this->get_best_trajectory(prev_s, prev_d, goal_s, goal_d,
                                        this->planning_horizon,
                                        other_vehicle_predict,
                                        this->print_cost);
  double trajectory_cost = free_flow.cost;
  while(trajectory_cost>1000 && speed_change>0.0) {
    speed_change -= 0.5;
    if(speed_change<0.0){
      speed_change = 0.0;
    }
    desired_speed = prev_s[1] + sign_speed_change*speed_change;
    goal_s = {
            (desired_speed + prev_s[1]) / 2 * this->planning_horizon + prev_s[0],
            desired_speed,
            (desired_speed - prev_s[1]) / this->planning_horizon};

    free_flow = this->get_best_trajectory(prev_s, prev_d, goal_s, goal_d,
                                          this->planning_horizon,
                                          other_vehicle_predict,
                                          this->print_cost);

    trajectory_cost = free_flow.cost;
  }
  if(free_flow.time<1){
    cout<<endl;
  }
//  cout << "speed_change = "<<speed_change<<", displacment = "<<desired_loc<<endl;
  return free_flow;
}

trajectory Vehicle::get_trajectory() {
  return this->car_trajectory;
}

vector<vector<double>> Vehicle::get_XY_trajectory(
        vector<double>& x_traj,
        vector<double>& y_traj,
        int path_point_size,
        double path_dt,
        double corse_path_dt,
        vector<float>& maps_s,
        vector<double>& maps_x,
        vector<double>& maps_y
){
  if(path_point_size>x_traj.size()){

    vector<double> s_traj, d_traj, t_traj, ref_x_traj, ref_y_traj;
    int prev_size = x_traj.size();
    if(prev_size>2){
      t_traj.push_back(-path_dt);
      ref_x_traj.push_back(x_traj[prev_size-2]);
      ref_y_traj.push_back(y_traj[prev_size-2]);

      t_traj.push_back(0);
      ref_x_traj.push_back(x_traj[prev_size-1]);
      ref_y_traj.push_back(y_traj[prev_size-1]);
    }
    for (int i = 0; i < planning_horizon/corse_path_dt-1; i++) {
      double t = (i+1) * corse_path_dt;
      double s_val = 0, d_val = 0;
      for (int j = 0; j < this->car_trajectory.coefficients_s.size(); j++) {
        s_val = helper.polyval(this->car_trajectory.coefficients_s,t);
        d_val = helper.polyval(this->car_trajectory.coefficients_d,t);
        s_val = fmod(s_val,this->track_length);
      }
      s_traj.push_back(s_val);
      d_traj.push_back(d_val);
      vector<double> xy_point = helper.getXY(s_val, d_val, maps_s, maps_x, maps_y);
      ref_x_traj.push_back(xy_point[0]);
      ref_y_traj.push_back(xy_point[1]);
      t_traj.push_back(t);
    }
    vector<vector<double>> s_diff_coeff = helper.get_N_derivatives(this->car_trajectory.coefficients_s,2);
    vector<vector<double>> d_diff_coeff = helper.get_N_derivatives(this->car_trajectory.coefficients_d,2);
    tk::spline s_x,s_y;
    s_x.set_points(t_traj,ref_x_traj);
    s_y.set_points(t_traj,ref_y_traj);
    double t = 0;
    for (int i = 0; i < path_point_size - prev_size+1; i++) {
      t = (i+1) * path_dt;
      double s_val,d_val;
      s_val = helper.polyval(this->car_trajectory.coefficients_s,t);
      d_val = helper.polyval(this->car_trajectory.coefficients_d,t);
      s_val = fmod(s_val,this->track_length);
      vector<double> xy_point = helper.getXY(s_val, d_val, maps_s, maps_x, maps_y);

      x_traj.push_back(s_x(t));
      y_traj.push_back(s_y(t));
    }
    x_traj.pop_back();
    y_traj.pop_back();
    if(s_traj[0]==0){
      cout << endl;
    }
    vector<double> s_terminal = helper.polyval_multi(s_diff_coeff,t - path_dt);
    vector<double> d_terminal = helper.polyval_multi(d_diff_coeff,t - path_dt);
    prev_s = s_terminal;
    prev_s[0] = fmod(prev_s[0],this->track_length);
    prev_d = d_terminal;
  }

  return {x_traj,y_traj};
}

void Vehicle::set_planning_horizon(double planning_horizon_) {
  this->planning_horizon = planning_horizon_;
}
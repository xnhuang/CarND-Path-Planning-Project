//
// Created by xnhuang on 8/30/17.
//

#ifndef PATH_PLANNING_ROAD_H
#define PATH_PLANNING_ROAD_H

#include "Helper.h"
#include "Vehicle.h"

class Vehicle;
using namespace std;

class Road {
  double free_distance;
  double delta_speed;
protected:
  vector<vector<Vehicle_Prediction>> lane_vehicle_container;
  double delay_time;
  vector<vector<double>> min_distance;
  vector<vector<int>> min_distance_id;
  vector<double> lane_speed;
  vector<double> lane_speed_rear;

public:
  Road(double free_distance_, double delta_speed_);
  ~Road();

  void update_road(vector<Vehicle_Prediction>& other_vehicle,vector<double>& previous_path_x, double path_dt, Vehicle& car);
  vector<Vehicle_Prediction> get_lane_vehicle(int lane);
  vector<double> get_lane_min_distance(int lane);
  vector<int> get_lane_min_distance_id(int lane);
  double get_lane_spd(int lane);

  bool busy_lane(Vehicle& car, int lane, double distance);
  bool free_lane(Vehicle& car,int lane);
  int lane_change_available(Vehicle& car);
};


#endif //PATH_PLANNING_ROAD_H

//
// Created by xnhuang on 8/30/17.
//

#include "Road.h"

Road::Road(double free_distance_, double delta_speed_){
  free_distance = free_distance_;
  delta_speed = delta_speed_;
};
Road::~Road() = default;

void Road::update_road(vector<Vehicle_Prediction>& other_vehicle,vector<double>& previous_path_x, double path_dt, Vehicle& car){
  delay_time = previous_path_x.size()*path_dt;
  double ego_loc = car.get_prev_s()[0];
  double spd_limit = car.get_speed_limit();

  vector<double> lane_speed_local = {spd_limit, spd_limit, spd_limit};
  vector<double> lane_speed_rear_local = {spd_limit, spd_limit, spd_limit};

  vector<double> min_distance_left = {999,999};
  vector<double> min_distance_center = {999,999};
  vector<double> min_distance_right = {999,999};

  int left_count = -1,center_count = -1,right_count = -1;
  vector<int> min_distance_left_id = {-1,-1};
  vector<int> min_distance_center_id = {-1,-1};
  vector<int> min_distance_right_id = {-1,-1};

  vector<Vehicle_Prediction> vehicle_left;
  vector<Vehicle_Prediction> vehicle_center;
  vector<Vehicle_Prediction> vehicle_right;

  for(int other_vehicle_id = 0;other_vehicle_id<other_vehicle.size();other_vehicle_id++){
    Vehicle_Prediction sensor_read  = other_vehicle[other_vehicle_id];
    sensor_read.set_initial(delay_time);

    if(sensor_read.get_lane()==0){
      vehicle_left.push_back(sensor_read);
      left_count++;
      double distance_to_ego = sensor_read.get_s()-ego_loc;
      if(distance_to_ego>=0){
        if(distance_to_ego<min_distance_left[0]){
          min_distance_left[0] = distance_to_ego;
          min_distance_left_id[0] = left_count;
          if(distance_to_ego<free_distance){
            lane_speed_local[0] = sensor_read.get_speed();
          }
        }
      }else{
        if((-distance_to_ego)<min_distance_left[1]){
          min_distance_left[1] = -distance_to_ego;
          min_distance_left_id[1] = left_count;
          if(distance_to_ego<free_distance){
            lane_speed_rear_local[0] = sensor_read.get_speed();
          }
        }
      }
    }
    if(sensor_read.get_lane()==1){
      vehicle_center.push_back(sensor_read);
      center_count++;
      double distance_to_ego = sensor_read.get_s()-ego_loc;
      if(distance_to_ego>=0){
        if(distance_to_ego<min_distance_center[0]){
          min_distance_center[0] = distance_to_ego;
          min_distance_center_id[0] = center_count;
          if(distance_to_ego<free_distance){
            lane_speed_local[1] = sensor_read.get_speed();
          }
        }
      }else{
        if((-distance_to_ego)<min_distance_center[1]){
          min_distance_center[1] = -distance_to_ego;
          min_distance_center_id[1] = center_count;
          if(distance_to_ego<free_distance){
            lane_speed_rear_local[1] = sensor_read.get_speed();
          }
        }
      }
    }
    if(sensor_read.get_lane()==2){
      vehicle_right.push_back(sensor_read);
      right_count++;
      double distance_to_ego = sensor_read.get_s()-ego_loc;
      if(distance_to_ego>=0){
        if(distance_to_ego<min_distance_right[0]){
          min_distance_right[0] = distance_to_ego;
          min_distance_right_id[0] = right_count;
          if(distance_to_ego<free_distance){
            lane_speed_local[2] = sensor_read.get_speed();
          }
        }
      }else{
        if((-distance_to_ego)<min_distance_right[1]){
          min_distance_right[1] = -distance_to_ego;
          min_distance_right_id[1] = right_count;
          if(distance_to_ego<free_distance){
            lane_speed_rear_local[2] = sensor_read.get_speed();
          }
        }
      }
    }
  }
  min_distance = {min_distance_left,min_distance_center,min_distance_right};
  min_distance_id = {min_distance_left_id,min_distance_center_id,min_distance_right_id};
  lane_vehicle_container = {vehicle_left, vehicle_center,vehicle_right};
  lane_speed = lane_speed_local;
  lane_speed_rear = lane_speed_rear_local;
};

vector<Vehicle_Prediction> Road::get_lane_vehicle(int lane){
  return lane_vehicle_container[lane];
};

vector<double> Road::get_lane_min_distance(int lane){
  return min_distance[lane];
};

vector<int> Road::get_lane_min_distance_id(int lane){
  return min_distance_id[lane];
};

double Road::get_lane_spd(int lane){
  return lane_speed[lane];
}

bool Road::busy_lane(Vehicle& car, int lane, double distance){
  int ego_lane = car.get_prev_lane();
  vector<double> min_distance_lane = min_distance[ego_lane];
  return min_distance_lane[0]<distance || this->get_lane_spd(lane)<car.get_speed_limit();
};

bool Road::free_lane(Vehicle& car,int lane){
  vector<double> min_distance_lane = min_distance[lane];
  bool front_safe = min_distance_lane[0]>free_distance*2/4;
  bool rear_safe = min_distance_lane[1]>free_distance*2/4
                   || (min_distance_lane[1]>free_distance*1/4 && lane_speed_rear[lane]<car.get_prev_s()[1]);
  return (front_safe && rear_safe);
};

int Road::lane_change_available(Vehicle& car){
  int ego_lane = car.get_prev_lane();
  int target_lane = ego_lane;
  double lane_spd_ego = lane_speed[ego_lane];
  if(ego_lane==0){
    if(this->free_lane(car,1) && lane_speed[1]>lane_spd_ego+delta_speed){
      target_lane = 1;
    }
  }
  if(ego_lane==1){
    if(this->free_lane(car,2) && lane_speed[2]>lane_spd_ego+delta_speed){
      target_lane = 2;
    }
    if(this->free_lane(car,0) && lane_speed[0]>lane_spd_ego+delta_speed){
      target_lane = 0;
    }
  }
  if(ego_lane==2){
    if(this->free_lane(car,1) && lane_speed[1]>lane_spd_ego+delta_speed){
      target_lane = 1;
    }
  }
  return target_lane;
};
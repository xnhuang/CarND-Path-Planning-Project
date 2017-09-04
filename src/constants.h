//
// Created by xnhuang on 8/19/17.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

#include<vector>

using namespace std;
const double VEHICLE_RADIUS = 1.5;              // meters

const int PREVIOUS_PATH_POINTS_TO_KEEP = 25;
const int NUM_PATH_POINTS = 50;
const double PATH_DT = 0.02;                    // seconds
const double PATH_GENERATION_DT = 0.8;
const double PREDICTION_HORIZON = 2.5;
const int PRINT_COST = 0;
const double TRACK_LENGTH = 6945.554;           // meters

// for trajectory generation/evaluation and non-ego car predictions
const int N_SAMPLES_PATH = 5;
const int PATH_GENERATION_POINT_N = 17;

// sigma values for perturbing targets

const double SIGMA_S1 = 10.0;                     // s
const double SIGMA_S_DOT = 4.0;                 // s_dot
const double SIGMA_S_DDOT = 2;                  // s
const double SIGMA_D1 = 1;                       // d
const double SIGMA_D_DOT = 1;                   // d_dot
const double SIGMA_D_DDOT = 1;                  // d_double_dot
const double SIGMA_T = 2;

const double MAX_INSTANTANEOUS_JERK = 9;       // m/s/s/s
const double MAX_INSTANTANEOUS_ACCEL = 8;      // m/s/s

const double EXPECTED_JERK_IN_ONE_SEC = 1.9;      // m/s/s
const double EXPECTED_ACC_IN_ONE_SEC = 0.9;       // m/s
const double SPEED_LIMIT = 45;               // mph
const double DELTA_SPEED = 1;                 // speed difference for lane change, mps

const double DESIRED_TIME_HEADWAY = 0.8;
const double CAR_FOLLOW_MIN_DISTANCE = 0;
const double MAX_CAR_FOLLOW_TIME_HEADWAY = 2.0;
const double FREE_DISTANCE = 40.0; //m

// cost function weights
const double TIME_DIFF_COST_WEIGHT = 10;
const double TRAJ_DIFF_COST_WEIGHT = 10;
const double COLLISION_COST_WEIGHT = 999999;
const double BUFFER_COST_WEIGHT = 100;
const double SPEED_LIMIT_COST_WEIGHT = 999999;
const double EFFICIENCY_COST_WEIGHT = 1000;
const double MAX_ACCEL_COST_WEIGHT = 9999;
const double AVG_ACCEL_COST_WEIGHT = 1000;
const double MAX_JERK_COST_WEIGHT = 9999;
const double AVG_JERK_COST_WEIGHT = 1000;

#endif //PATH_PLANNING_CONSTANTS_H


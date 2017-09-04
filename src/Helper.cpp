//
// Created by xnhuang on 8/8/17.
//

#include "Helper.h"

Helper::Helper(){};
Helper::~Helper(){};

// For converting back and forth between radians and degrees.
double Helper::pi() { return M_PI; }

double Helper::deg2rad(double x) { return x * pi() / 180; }

double Helper::rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.

string Helper::hasData(string s) {
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

double Helper::distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int Helper::ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int Helper::NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Helper::getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Helper::getXY(double s, double d, vector<float>& maps_s, vector<double>& maps_x, vector<double>& maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

double Helper::mph2mps(double mph){
  return 0.447*mph;
};



double Helper::logistic(double x) {
  return 2.0 / (1 + exp(-x)) - 1.0;
}

double Helper::polyval(vector<double>& coefficients, double x){
  double total = 0.0;
  for(int order = 0;order<coefficients.size();order++){
    total += coefficients[order]*pow(x,order);
  }
  return total;
};

vector<double> Helper::polyval_multi(vector<vector<double>>& coefficients_collection, double x){
  vector<double> output_;
  for(vector<double>& coefficients:coefficients_collection){
    double poly_value = polyval(coefficients,x);
    output_.push_back(poly_value);
  };
  return output_;
};

vector<double> Helper::differentiate(vector<double> coefficients){
  vector<double> new_cos;
  for(int order = 1;order<coefficients.size();order++){
    new_cos.push_back(order*coefficients[order]);
  }
  return new_cos;
};

double Helper::nearest_approach(trajectory& candidate_traj, Vehicle_Prediction& candidate_vehicle){
  double closest = 999999;
  vector<double> s = candidate_traj.coefficients_s;

  for(int i=0;i<100;i++){
    double t = float(i) / 100 * candidate_traj.time;
    double cur_s = polyval(candidate_traj.coefficients_s,t);
    double cur_d = polyval(candidate_traj.coefficients_d,t);
    vector<double> predict_state = candidate_vehicle.predict(t);

    double dist = sqrt(pow((cur_s-predict_state[0]),2) + pow((cur_d-predict_state[3]),2));
    if (dist < closest){
      closest = dist;
    }
  }
  return closest;
};

double Helper::nearest_approach_to_any_vehicle(trajectory& candidate, vector<Vehicle_Prediction>& vehicle_list){
  double closest = 999999;
  for(Vehicle_Prediction& vehicle:vehicle_list){
    double d = nearest_approach(candidate,vehicle);
    if(d<closest){
      closest = d;
    }
  }
  return closest;
};

vector<vector<double>> Helper::get_N_derivatives(vector<double> coefficients, int N){
  vector<vector<double>> coefficients_collection;
  coefficients_collection.push_back(coefficients);
  for(int i = 0;i<N;i++){
    if(!coefficients.empty()){
      coefficients = differentiate(coefficients);
      coefficients_collection.push_back(coefficients);
    }
  };
  return coefficients_collection;
};

//void Helper::show_trajectory(vector<double> s_coeffs, vector<double> d_coeffs, double time_desire){
//  vector<double> time;
//  vector<pair<double, double> > plot_s;
//  vector<pair<double, double> > plot_d;
//  for(double t=0.0;t<time_desire+0.01;t+=0.25){
//    time.push_back(t);
//    plot_s.push_back(make_pair(t,polyval(s_coeffs,t)));
//    plot_d.push_back(make_pair(t,polyval(d_coeffs,t)));
//  };
//  Gnuplot plot;
//  plot << "plot '-' with vectors title 'plot_s', "
//       << "'-' with vectors title 'plot_d'\n";
//  plot.send1d(plot_s);
//  plot.send1d(plot_d);
//};
//
//void Helper::show_trajectory(vector<double> s_coeffs, vector<double> d_coeffs, double time_desire, Vehicle_Prediction vehicle){
//  vector<double> time;
//  vector<pair<double, double> > s;
//  vector<pair<double, double> > d;
//  vector<pair<double, double> > s_veh;
//  vector<pair<double, double> > d_veh;
//  for(double t=0.0;t<time_desire+0.01;t+=0.25){
//    time.push_back(t);
//    s.push_back(make_pair(t,polyval(s_coeffs,t)));
//    d.push_back(make_pair(t,polyval(d_coeffs,t)));
//    vector<double> vehicle_state = vehicle.predict(t);
//    s_veh.push_back(make_pair(t,vehicle_state[0]));
//    d_veh.push_back(make_pair(t,vehicle_state[3]));
//  };
//  Gnuplot plot1;
//  plot1 << "plot '-' with lines title 's', "
//        << "'-' with lines title 's_veh'\n";
//  plot1.send1d(s);
//  plot1.send1d(s_veh);
//
//  Gnuplot plot2;
//  plot2 << "set yrange [-0.5:4.5]\n";
//  plot2 << "plot '-' with lines title 'd', "
//        << "'-' with lines title 'd_veh'\n";
//  plot2.send1d(d);
//  plot2.send1d(d_veh);
//};


vector<double> Helper::interpolate_points(vector<double> pts_x, vector<double> pts_y,
                                  double interval, int output_size) {
  // uses the spline library to interpolate points connecting a series of x and y values
  // output is output_size number of y values beginning at y[0] with specified interval

  if (pts_x.size() != pts_y.size()) {
    cout << "ERROR! SMOOTHER: interpolate_points size mismatch between pts_x and pts_y" << endl;
    return { 0 };
  }

  tk::spline s;
  s.set_points(pts_x,pts_y);    // currently it is required that X is already sorted
  vector<double> output;
  for (int i = 0; i < output_size; i++) {
    if(pts_x[0] + i * interval>0){
      output.push_back(s(pts_x[0] + i * interval));
    }
  }
  return output;
}

Vehicle_Prediction::Vehicle_Prediction(vector<double> start_,string type){
  if(type.compare("xy")==0){
    start_frenet.push_back(start_[5]);
    start_frenet.push_back(sqrt(pow(start_[3],2)+pow(start_[4],2)));
    start_frenet.push_back(0);
    start_frenet.push_back(start_[6]);
    start_frenet.push_back(0);
    start_frenet.push_back(0);
  }
  else{
    start_frenet = start_;
  }


  lane = (int)((start_frenet[3])/4);
};

Vehicle_Prediction::~Vehicle_Prediction() = default;

vector<double> Vehicle_Prediction::predict(double t) {
  vector<double> state;
  vector<double>::const_iterator first = start_frenet.begin();
  vector<double>::const_iterator mid = start_frenet.begin()+3;
  vector<double>::const_iterator last = start_frenet.end();
  vector<double> s(first, mid);
  vector<double> d(mid, last);

  state.push_back(s[0] + (s[1] * t) + s[2] * t*t / 2.0);
  state.push_back(s[1] + s[2] * t);
  state.push_back(s[2]);
  state.push_back(d[0] + (d[1] * t) + d[2] * t*t / 2.0);
  state.push_back(d[1] + d[2] * t);
  state.push_back(d[2]);

  return state;
};

int Vehicle_Prediction::get_lane() {
  return this->lane;
}

double Vehicle_Prediction::get_s() {
  return this->start_frenet[0];
}

void Vehicle_Prediction::set_initial(double t) {
  start_frenet = this->predict(t);
}

double Vehicle_Prediction::get_speed() {
  return this->start_frenet[1];
}
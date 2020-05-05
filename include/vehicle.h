#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int lane, float s, float v, float a, string state);
  Vehicle(const Vehicle& v);
  Vehicle& operator=( const Vehicle& v );
  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> &predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, 
                                      map<int, vector<Vehicle>> &predictions);

  vector<float> get_kinematics(map<int, vector<Vehicle>> &predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> &predictions);

  vector<Vehicle> lane_change_trajectory(string state, 
                                         map<int, vector<Vehicle>> &predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, 
                                              map<int, vector<Vehicle>> &predictions);

  void increment(int dt);

  float position_at(int t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane, 
                          Vehicle &rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane, 
                         Vehicle &rVehicle);

  vector<Vehicle> generate_predictions(int horizon = 2);

  void realize_next_state(vector<Vehicle> &trajectory);

  void configure(vector<int> &road_data);

  // public Vehicle variables
  struct collider{
    bool collision; // is there a collision?
    int  time; // time collision happens
  };

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, 
                                     {"LCR", 1}, {"PLCR", 1}};

  vector<int> LANE_SPEEDS = {70,50,60};

  int preferred_buffer; // impacts "keep lane" behavior.

  int lane, goal_lane, goal_s, previous_lane;
  int lanes_available = 3;

  float v;
  float target_speed;
  float a, max_acceleration;
  
  float s, d, x, y, yaw;
  string state;

  void update(double x, double y, double s, double d, double speed, double yaw, string state);
};

#endif  // VEHICLE_H
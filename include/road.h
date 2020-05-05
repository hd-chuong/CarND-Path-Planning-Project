#ifndef ROAD_H
#define ROAD_H

#include <map>
#include <string>
#include <vector>
#include "vehicle.h"

class Road {
 public:
  const int LANE_WIDTH = 4;
  // Constructor
  Road(int speed_limit, double traffic_density, std::vector<int> &lane_speeds);

  // Destructor
  virtual ~Road();

  // Road functions
  Vehicle get_ego();

  void populate_traffic();
  void update_vehicle(vector<vector<double>> & vehicle_data, int prev_size);
  void advance();
  void localize_ego(int s, int d);
  void display(int timestep);
  void set_vehicles(vector<Vehicle> & vehicles, vector<int> & new_ids);
  void add_ego(int lane_num, int s, std::vector<int> &config_data);

  void cull();

  // Road variables
  int update_width = 70;

  int vehicles_added = 0;

  int ego_key = -1;

  int num_lanes, speed_limit, camera_center;

  double density; 

  std::map<int, Vehicle> vehicles;

  std::string ego_rep = " *** ";

  std::vector<int> lane_speeds; 

  // some helper function
  int get_lane(double d);

  map<int, vector<Vehicle> > predict();
};

#endif  // ROAD_H
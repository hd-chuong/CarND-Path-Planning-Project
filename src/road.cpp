#include <iostream>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "vehicle.h"
#include "road.h"

using std::vector;
using std::map;
using std::string;


// Initializes Road

Road::Road(int speed_limit, double traffic_density, vector<int> &lane_speeds) {
  this->num_lanes = lane_speeds.size();
  this->lane_speeds = lane_speeds;
  this->speed_limit = speed_limit;
  this->density = traffic_density;
  this->camera_center = this->update_width/2;
}

Road::~Road() {}

Vehicle Road::get_ego() {

  Vehicle ego = this->vehicles.find(this->ego_key)->second;
  //std::cout << "inside road.cpp " << ego.state << std::endl;
  return ego;
}

void Road::localize_ego(int s, int d)
{
  Vehicle & ego = this->vehicles.find(this->ego_key)->second;
  ego.s = s;
  ego.d = d;
}

void Road::set_vehicles(vector<Vehicle> & new_vehicles, vector<int> & new_ids)
{
  Vehicle ego = get_ego();
  vehicles.clear();
  vehicles[ego_key] = ego;
  for (int i = 0; i < new_ids.size(); ++i)
  {
    int id =  new_ids[i];
    vehicles[id] = new_vehicles[i];
  }
}
// void Road::populate_traffic() {
//   int start_s = std::max(this->camera_center - (this->update_width/2), 0);

//   for (int l = 0; l < this->num_lanes; ++l) {
//     int lane_speed = this->lane_speeds[l];
//     bool vehicle_just_added = false;

//     for (int s = start_s; s < start_s+this->update_width; ++s) {
//       if (vehicle_just_added) {
//         vehicle_just_added = false;
//       }
      
//       if (((double) rand() / (RAND_MAX)) < this->density) {
//         Vehicle vehicle = Vehicle(l,s,lane_speed,0);
//         vehicle.state = "CS";
//         this->vehicles_added += 1;
//         this->vehicles.insert(std::pair<int,Vehicle>(vehicles_added,vehicle));
//         vehicle_just_added = true;
//       }
//     }
//   }
// }

map<int, vector<Vehicle>> Road::predict()
{
  map<int, vector<Vehicle>> predictions;
  for (map<int,Vehicle>::iterator it = this->vehicles.begin(); it != vehicles.end(); ++it)
  {
    int id = it->first;
    if (id == ego_key) continue;
    vector<Vehicle> car_next_states = it->second.generate_predictions(1);
    predictions[id] = car_next_states;
  }
  //std::cout << "Number of surrounding cars: " << predictions.size() << std::endl;
  return predictions;
}

// void Road::advance() {
//   map<int ,vector<Vehicle> > predictions;

//   map<int, Vehicle>::iterator it = this->vehicles.begin();

//   while (it != this->vehicles.end()) {
//     int v_id = it->first;
//     vector<Vehicle> preds = it->second.generate_predictions();
//     predictions[v_id] = preds;
//     ++it;
//   }
  
//   it = this->vehicles.begin();

//   while (it != this->vehicles.end()) 
//   {
//     int v_id = it->first;
//     if (v_id == ego_key) {   
//       vector<Vehicle> trajectory = it->second.choose_next_state(predictions);
//       it->second.realize_next_state(trajectory);
//     } else {
//       it->second.increment(1);
//     }
//     ++it;
//   }   
// }

void Road::add_ego(int lane_num, int s, vector<int> &config_data) {
  map<int, Vehicle>::iterator it = this->vehicles.begin();

  while (it != this->vehicles.end()) {
    int v_id = it->first;
    Vehicle v = it->second;
    if (v.lane == lane_num && v.s == s) {
      this->vehicles.erase(v_id);
    }
    ++it;
  }
    
  Vehicle ego = Vehicle(lane_num, s, this->lane_speeds[lane_num], 0, "KL");
  ego.configure(config_data);
  this->vehicles.insert(std::pair<int,Vehicle>(ego_key,ego));
}

int Road::get_lane(double d)
{
  return (int) d / LANE_WIDTH;
}

// void Road::display(int timestep) {
//   Vehicle ego = this->vehicles.find(this->ego_key)->second;
//   int s = ego.s;
//   string state = ego.state;

//   this->camera_center = std::max(s, this->update_width/2);
//   int s_min = std::max(this->camera_center - this->update_width/2, 0);
//   int s_max = s_min + this->update_width;

//   vector<vector<string>> road;

//   for (int i = 0; i < this->update_width; ++i) {
//     vector<string> road_lane;
//     for (int ln = 0; ln < this->num_lanes; ++ln) {
//       road_lane.push_back("     ");
//     }
//     road.push_back(road_lane);
//   }

//   map<int, Vehicle>::iterator it = this->vehicles.begin();

//   while (it != this->vehicles.end()) {
//     int v_id = it->first;
//     Vehicle v = it->second;

//     if (s_min <= v.s && v.s < s_max) {
//       string marker = "";

//       if (v_id == this->ego_key) {
//         marker = this->ego_rep;
//       } else {
//         std::stringstream oss;
//         std::stringstream buffer;
//         buffer << " ";
//         oss << v_id;

//         for (int buffer_i = oss.str().length(); buffer_i < 3; ++buffer_i) {
//           buffer << "0";
//         }
//         buffer << oss.str() << " ";
//         marker = buffer.str();
//       }
//       road[int(v.s - s_min)][int(v.lane)] = marker;
//     }
//     ++it;
//   }
    
//   std::ostringstream oss;
//   oss << "+Meters ======================+ step: " << timestep << std::endl;
//   int i = s_min;

//   for (int lj = 0; lj < road.size(); ++lj) {
//     if (i%20 ==0) {
//       std::stringstream buffer;
//       std::stringstream dis;
//       dis << i;
      
//       for (int buffer_i = dis.str().length(); buffer_i < 3; ++buffer_i) {
//         buffer << "0";
//       }
      
//       oss << buffer.str() << dis.str() << " - ";
//     } else {
//       oss << "      ";
//     }          
//     ++i;
//     for (int li = 0; li < road[0].size(); ++li) {
//       oss << "|" << road[lj][li];
//     }
//       oss << "|";
//       oss << "\n";
//   }

//   std::cout << oss.str();
// }

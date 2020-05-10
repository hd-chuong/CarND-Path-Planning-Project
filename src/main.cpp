//cur issue: too close in the old lane to turn
//speed varies lots although modifying

#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "vehicle.h"
#include "road.h"
#include "jerk.h"
#include "helpers.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

const int EGO_KEY = -1;

double max_s = 1000000.0;

double SPEED_LIMIT = 22;

vector<double> LANE_SPEEDS = {SPEED_LIMIT,SPEED_LIMIT,SPEED_LIMIT};

double TRAFFIC_DENSITY = 0.15;

double MAX_ACCEL = 9;

double goal_s = max_s;

// implement (1 - e^(-x)) / (1 + e^x)
double delta_sigmoid(double x)
{
  return (1 - exp(- x)) / (1 + exp(- x));
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  bool justStart = true;
  //
  //
  int lane = 1;
  int num_lanes = LANE_SPEEDS.size();
  vector<double> ego_config = {SPEED_LIMIT, goal_s, MAX_ACCEL};
  //
  //
  Road road = Road(SPEED_LIMIT, TRAFFIC_DENSITY, LANE_SPEEDS);

  road.add_ego(lane, 0, ego_config);

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0

  double ref_speed = 0;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,
                &map_waypoints_y,
                &map_waypoints_s,
               &map_waypoints_dx,
               &map_waypoints_dy,
               &lane, &justStart, &road]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          car_speed = car_speed / 2.237;
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

        //double ref_vel = car_speed;

        //int prev_size = previous_path_x.size();

          Vehicle ego = road.get_ego();
          
          //std::cout << "early main.cpp ego state " << ego.state << " s = " << ego.s << " v = " << ego.v << std::endl;
          //std::cout << "sensor value: s = " << car_s << " v = " << car_speed << std::endl;          
          ego.update(car_x, car_y, car_s, car_d, car_speed, car_yaw, ego.state);
          //std::cout << "Early buffer " << ego.preferred_buffer << std::endl;
          vector<Vehicle> other_cars;
          vector<int> other_ids;
          for (int i = 0; i < sensor_fusion.size(); ++i)
          {
            other_ids.push_back(sensor_fusion[i][0]);
            double x = sensor_fusion[i][1];
            double y = sensor_fusion[i][2];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double s = sensor_fusion[i][5];
            double d =sensor_fusion[i][6];
            double v = sqrt(vx*vx+vy*vy);
            //std::cout << v << std::endl;
            Vehicle new_car;
            new_car.update(x, y, s, d, v, atan2(vy, vx), (string) "KL");
            other_cars.push_back(new_car);
            // if (sqrt(pow(x-car_x, 2) + pow(y-car_y,2)) < 2 && (int) car_d / 4 == (int) d / 4) {
            //   std::cout << "collision ego " << car_s << " " << s;
            //   throw "fail";
            // }
            // //std::cout << "s: " << s << " d: " << d << std::endl;
          }
          road.set_vehicles(other_cars, other_ids);

          map<int, vector<Vehicle>> other_car_predictions = road.predict();

          // behavior planner
          vector<Vehicle> ego_best_state = ego.choose_next_state(other_car_predictions);
          
          // trajectory generator

          int next_lane = ego_best_state[1].lane;
          ego.state = ego_best_state[1].state;
          double next_d, next_s, next_speed, target_speed;
          //std::cout << "buffer ego " << ego.preferred_buffer << "other buffer " << other_cars[0].preferred_buffer << std::endl;
          if (ego.state == "PLCL")
          {
            next_d = next_lane * 4 + 1.5;
            //next_s = ego_best_state[1].s;
            target_speed = ego.get_kinematics(other_car_predictions, ego.lane)[1];
          }
          else if (ego.state == "PLCR")
          {
            next_d = next_lane * 4 + 2.5;
            //next_s = ego_best_state[1].s;
            target_speed = ego.get_kinematics(other_car_predictions, ego.lane)[1];
          }
          else if (ego.state == "LCL" || ego.state == "LCR")
          {
            next_d = next_lane * 4 + 2;
            target_speed = ego.get_kinematics(other_car_predictions, ego.lane)[1];
          }
          else 
          {
            next_d = next_lane * 4 + 2; 
            //next_s = ego_best_state[1].s;
            target_speed = ego_best_state[1].v;
          }
          // std::cout <<" current speed " << ego.v <<  " target speed" << target_speed << std::endl;
          std::cout << "delta coeff " << delta_sigmoid(target_speed - ego.v) << std::endl;
          next_speed = ego.v + delta_sigmoid(target_speed - ego.v) * ego.max_acceleration;
          // if (ego.v <= target_speed)
          // {
          //   next_speed = std::min(SPEED_LIMIT, ego.v + 0.224);
          // }
          // else {
          //   next_speed = std::max(2.0, ego.v - 0.224);
          // }
          // Vehicle dummyVehicle;
          // if (!ego.get_vehicle_ahead(other_car_predictions, ego.lane, dummyVehicle))
          // {
          //   rel_speed = std::min(SPEED_LIMIT, rel_speed + 0.224);
          // }
          // else {
          //   rel_speed = std::max(2.0, rel_speed - 0.224);
          // }
          //std::cout << "real next d " << next_d << " lane. " << next_lane << " real next speed " << next_speed << std::endl;
          next_s = ego.s + (ego.v + next_speed) / 2; 
          ego.lane = next_lane;
          //next_speed = rel_speed;
          if (justStart) 
          {
            next_s += 20.0 / 2.24; 
            justStart = false;
          }
          //std::cout << "cur s " << ego.s << " next s " << next_s << std::endl;
          road.vehicles[EGO_KEY] = ego;
          road.vehicles[EGO_KEY].state = ego.state;
                    
          int next_wp = -1;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          vector<double> ptsx;
          vector<double> ptsy;
          //std::cout << "prev size " <<prev_size << std::endl; 
          
          int prev_size = std::min((int) previous_path_x.size(), 10);

          if ( prev_size < 2 ) {
              // There are not too many...
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
          } else {
              // Use the last two points.
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
          }

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //vector<double> raw_traj_x, raw_traj_y;

          vector<double> targetXY = getXY(next_s + 25, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(next_s + 50, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp3 = getXY(next_s + 75, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          double target_x = targetXY[0];
          double target_y = targetXY[1];

          // cout << "-------------------------------" << endl;
          // cout << "car value   : x = " << car_x << ", y = " << car_y << ", car_s = " << car_s << ", car_d = " << car_d << endl;
          // cout << "target value: x = " << target_x << ", y = " << target_y << ", target_s = " << next_s << ", target_d = " << next_d << endl;
          // cout << "-------------------------------" << endl;
          
          // ptsx.push_back(car_x);
          // ptsx.push_back(mid_x);
          // ptsx.push_back(target_x);

          // ptsy.push_back(car_y);
          // ptsy.push_back(mid_y);
          // ptsy.push_back(target_y);

          ptsx.push_back(target_x);
          ptsx.push_back(next_wp2[0]);
          ptsx.push_back(next_wp3[0]);

          ptsy.push_back(target_y);
          ptsy.push_back(next_wp2[1]);
          ptsy.push_back(next_wp3[1]);

          // use the coordinate of car
          for (int i = 0; i < ptsx.size(); ++i)
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }
          
          // cout << "point (x,y) (not ship)" << endl;
          // cout << "------------------------------------------" << endl;
          // for (int i = 0; i < ptsx.size(); ++i)
          // {
          //   cout << "x = " << ptsx[i] << ", y = " << ptsy[i] << endl;
          // }
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // std::cout << "Test spline " << std::endl;
          // for (int i = 0; i < ptsx.size(); ++i)
          // {
          //   std::cout << ptsx[i] << " ";
          // }

          cout << std::endl;
          cout << "------------------------------------------" << endl;
          tk::spline smooth;
          smooth.set_points(ptsx, ptsy);
          
          for (int i = 0; i < prev_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }          
          
          vector<double> target_point = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          // double last_x, last_y;
          // if (prev_size > 0) {
          //   last_x = previous_path_x[prev_size - 1];
          //   last_y = previous_path_y[prev_size - 1];
          // } 
          // else {
          //   last_x = car_x;
          //   last_y = car_y;
          // }

          // double x_dist = target_x - last_x;
          // double y_dist = target_y - last_y;
          //std::cout << "next_speed " << next_speed << std::endl;
          double x_next = 30;
          double y_next = smooth(x_next);
          double target_dist = sqrt(x_next * x_next + y_next * y_next);
          double N = target_dist / (0.02 * next_speed)  ;
          //std::cout << "N = " << N << " target dist " << target_dist << std::endl;
          int number_point_left = prev_size;

          //std::cout << x_next << " " << y_next << std::endl;
          
          double inc_speed = 0;
          if (50 - prev_size > 0)
          {
            double speed_diff = next_speed - car_speed;
            inc_speed = speed_diff / (50 - prev_size);

          }

          double instant_speed = car_speed;
          double x_continue = 0;
          for (int i = 1; i <= 50 - prev_size /*&& number_point_left < 40*/; ++i)
          {
            instant_speed = car_speed + inc_speed * i;
            // double x_point = x_next / N * i;
            // double x_point = 0.02 * next_speed * i;
           double x_point = x_continue + instant_speed * 0.02;
            double y_point = smooth(x_point);
            x_continue = x_point;
            double x_ref = x_point;
            double y_ref = y_point;

            // transforming back
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          // cout << "***************Point debug****************" << endl;
          // for (int i = 0; i < next_x_vals.size(); ++i)
          // {
          //   cout << "(" << next_x_vals[i] << ", " << next_y_vals[i] << ")\n";
          // }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
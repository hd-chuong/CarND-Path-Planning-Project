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

double max_s = 6945.554;

int SPEED_LIMIT = 49;

vector<int> LANE_SPEEDS = {49,49,49};

double TRAFFIC_DENSITY = 0.15;

int MAX_ACCEL = 10;

double goal_s = max_s;
int goal_lane = 1;

Road road = Road(SPEED_LIMIT, TRAFFIC_DENSITY, LANE_SPEEDS);

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
  vector<int> ego_config = {SPEED_LIMIT, num_lanes, (int) goal_s, goal_lane, MAX_ACCEL};
  //
  //
  road.add_ego(lane, 0, ego_config);

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0


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
               &lane, &justStart]
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

        vector<double> ptsx;
        vector<double> ptsy;
        double ref_vel = car_speed;

        int prev_size = previous_path_x.size();
          
        //   if (prev_size > 0)
        //   {
        //     car_s = end_path_s;
        //   }

          Vehicle ego = road.get_ego();
          ego.update(car_x, car_y, car_s, car_d, car_speed, car_yaw);
          
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
            Vehicle new_car;
            new_car.update(x, y, s, d, v, atan2(vy, vx));
            other_cars.push_back(new_car);
          }
          road.set_vehicles(other_cars, other_ids);

          map<int, vector<Vehicle>> other_car_predictions = road.predict();

          vector<Vehicle> ego_best_state = ego.choose_next_state(other_car_predictions);
          int next_lane = ego_best_state[0].lane;
          double next_d = next_lane * 4 + 2;
          double next_s = ego_best_state[0].s + (justStart ? 10 : 0);
          float next_speed = ego_best_state[0].v;
          float next_a = ego_best_state[0].a;
          justStart = false;
          vector<double> begin_state = {car_s, car_speed, 0};
          vector<double> target_state = {(double) next_s, (double) next_speed, next_a};
          
          std::cout << "Current: Lane " << lane << ". d = " << car_d << ". s = " << car_s << " Speed =  " << car_speed << std::endl;
          std::cout << "Predicted: Lane " << next_lane << ". d = " << next_d << ". s = " << next_s << " Speed =  " << next_speed << "next acceleration " << next_a << std::endl;
          
          
          // bool too_close = false;

          // for (int i = 0; i < sensor_fusion.size(); ++i)
          // {
          //   float d = sensor_fusion[i][0];
          //   if ((d < 2 + 4*lane + 4 + 2) && (d > 4*lane + 4) )
          //   {
          //     double vx = sensor_fusion[i][3];
          //     double vy = sensor_fusion[i][4];

          //     double check_speed = sqrt(vx*vx + vy*vy);
          //     double check_car_s = sensor_fusion[i][5];

          //     check_car_s += ((double) prev_size * .02 * check_speed);

          //     if ((check_car_s > car_s) && (check_car_s - car_s < 30))
          //     {
          //       //ref_vel = 29.5;
          //       too_close = true;
          //       //std::
          //     }
          //   }
          // // }

          // if (too_close)
          // {
          //   ref_vel -= .224;
          // } else if (ref_vel < 49.5)
          // {
          //   ref_vel += .224;
          // }

//          Jerk traj_gen_s = Jerk(begin_state, target_state, 1.0);
//          Jerk traj_gen_d = Jerk({car_d, 0, 0}, {(double) next_d, 0, 0}, 1.0);
          int next_wp = -1;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // if (prev_size <= 1)
          // {
          //   double prev_car_x = car_x - cos(car_yaw);
          //   double prev_car_y = car_y - sin(car_yaw);

          //   ptsx.push_back(prev_car_x);          
          //   ptsx.push_back(car_x);

          //   ptsy.push_back(prev_car_y);
          //   ptsy.push_back(car_y);
          // }
          // else {
          //   // create a tangent line
          //   ref_x = previous_path_x[prev_size - 1];
          //   ref_y = previous_path_y[prev_size - 1];

          //   double ref_x_prev = previous_path_x[prev_size - 2];
          //   double ref_y_prev = previous_path_y[prev_size - 2];
 
          //   ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
          //   ptsx.push_back(ref_x_prev);
          //   ptsx.push_back(ref_x);

          //   ptsy.push_back(ref_y_prev);
          //   ptsy.push_back(ref_y);
          // }
          // for (int i = 0; i < 3; ++i)
          // {
          //   double time = 1.0 / 3 * i;
          //   //vector<double> waypoint = getXY(traj_gen_s.at(time), traj_gen_d.at(time), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          //   //std::cout << i << ": x = " << waypoint[0] << ", y = " << waypoint[1] << std::endl; 
          //   //ptsx.push_back(waypoint[0]);
          //   //ptsy.push_back(waypoint[1]);
          // }
          
          
          // vector<double> next_wp0 = getXY(car_s + 30, 2.0 +4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          // vector<double> next_wp1 = getXY(car_s + 60, 2.0 +4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          // vector<double> next_wp2 = getXY(car_s + 90, 2.0 +4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          // ptsx.push_back(next_wp0[0]);
          // ptsx.push_back(next_wp1[0]);
          // ptsx.push_back(next_wp2[0]);

          // ptsy.push_back(next_wp0[1]);
          // ptsy.push_back(next_wp1[1]);
          // ptsy.push_back(next_wp2[1]);

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          vector<double> raw_traj_x, raw_traj_y;

          vector<double> targetXY = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          double target_x = targetXY[0];
          double target_y = targetXY[1];

          double mid_x = 0.5 * (car_x + target_x);
          double mid_y = 0.5 * (car_y + target_y);
          
          cout << "-------------------------------" << endl;
          cout << "car value   : x = " << car_x << ", y = " << car_y << endl;
          cout << "target value: x = " << target_x << ", y = " << target_y << endl;
          cout << "-------------------------------" << endl;
          ptsx.push_back(0);
          ptsx.push_back(mid_x);
          ptsx.push_back(target_x);

          ptsy.push_back(0);
          ptsy.push_back(mid_y);
          ptsy.push_back(target_y);

          for (int i = 0; i < ptsx.size(); ++i)
          {
            //shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }
          
          cout << "Debug" << endl;
          cout << "------------------------------------------" << endl;
          for (int i = 0; i < ptsx.size(); ++i)
          {
            cout << "x = " << ptsx[i] << ", y = " << ptsy[i] << endl;
          }
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          cout << "------------------------------------------" << endl;
          tk::spline smooth;
          smooth.set_points(ptsx, ptsy);
          
          for (int i = 0; i < previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }          
          
          vector<double> target_point = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          double last_x;
          if (prev_size > 0)
          {
            last_x = previous_path_x[prev_size - 1];
          }
          else
          {
            last_x = car_x;
          }

          double x_dist = target_x - last_x;

          for (int i = 1; i <= 50 - previous_path_x.size(); ++i)
          {
            // double N = x_dist / (0.02 * ref_vel / 2.24);
            //double x_point = x_add_on + target_x / N;
            // double y_point = s(x_point);
            // x_add_on = x_point;
            double x_step = x_dist * 1.0 / (50 - previous_path_x.size()); 
            double x_point = last_x + x_step* i;
            double y_point = smooth(x_point);

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
          
          // vector<double> mock_x_vals, mock_y_vals;
          // double dist_inc = 0.3;
          // for (int i = 0; i < 50; ++i) {
          //   double next_s = car_s + (i+1)*dist_inc;
          //   double next_d = 6;
            
          //   vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          //   mock_x_vals.push_back(xy[0]);
          //   mock_y_vals.push_back(xy[1]);
          // }
          // next_x_vals = mock_x_vals;
          // next_y_vals = mock_y_vals;
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
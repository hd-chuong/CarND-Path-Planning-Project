#include <string>
#include <vector>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <functional>
#include <iostream>
using std::string;
using std::vector;
using std::map;

const double EFFICIENCY = 1;
const double ADVANCE = 10;
const double REACH_GOAL = 0;
const double STABILITY = 1;


// float lane_speed(const map<int, vector<Vehicle>> &predictions, int lane)
// {
//     for (auto it = predictions.begin(); it != predictions.end(); ++it)
//     {
//         int key = it->first;
//         Vehicle vehicle = it->second[0];
//         if (vehicle.lane == lane && key != -1)
//         {
//             return vehicle.v;
//         }
//     }
//     return -1;
// }

// place an expensive penalty if the vehicle does not move or move backwards
float advance_cost(const Vehicle & vehicle,
                    const vector<Vehicle> & trajectory,
                    const map<int, vector<Vehicle>> & predictions,
                    map<string, float> & data)
{
    double cur_s = vehicle.s;
    double expected_s = trajectory[0].s;
    double future_s = trajectory[1].s;
    //double distance = fabs(expected_s - cur_s) + fabs(future_s - cur_s);
    double distance = 0.0;

    if (expected_s < cur_s) distance += 1;
    if (future_s < cur_s) distance += 1;

    return distance / 2;
    //return distance < 1e-3 ? 1e3 : 1 / distance; 
}

float inefficiency_cost(const Vehicle &vehicle,
                        const vector<Vehicle> & trajectory,
                        const map<int, vector<Vehicle>> & predictions,
                        map<string, float> & data
                        )
{
    // cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
    //You can use the lane_speed function to determine the speed for a lane.
    //This function is very similar to what you have already implemented in the "Implement a Second Cost Function in C++" quiz
    // float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
    // if (proposed_speed_intended < 0)
    // {
    //     proposed_speed_intended = vehicle.target_speed;
    // }

    // float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
    // if (proposed_speed_final < 0)
    // {
    //     proposed_speed_final = vehicle.target_speed;
    // }

    // float cost = (2.0 * vehicle.target_speed - proposed_speed_intended - proposed_speed_final) / vehicle.target_speed;
    // std::cout << "Target speed " << vehicle.target_speed << " . Intended speed " << proposed_speed_intended << ". Final speed " << proposed_speed_final;
    // std::cout << ". Inefficient cost " << cost << std::endl;
    float possible_speed = trajectory[1].v;
    //std::cout << "Possible speed = " << possible_speed;
    float cost = 2 * vehicle.target_speed - possible_speed - trajectory[0].v;
    std::cout << "cur lane " << vehicle.lane << " target speed " << vehicle.target_speed << " . possible speed " << possible_speed << " current speed " << trajectory[0].v << std::endl;
    return cost;
}

float goal_distance_cost(const Vehicle & vehicle,
                        const vector<Vehicle> &trajectory,
                        const map<int, vector<Vehicle>> &predictions,
                        map<string, float> &data
                        )
{
    // Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
    // Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
    // This function is very similar to what you have already implemented in th "Implement a Cost Function in C++" quiz.    
    float cost;
    float distance = data["distance_to_goal"];
    if (distance > 0)
    {
        cost = 1 - 2*exp(-(fabs(2.0 * vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
    }
    else
    {
        cost = 1;
    }
    return cost;
}

float stability_cost(const Vehicle & vehicle,
                        const vector<Vehicle> &trajectory,
                        const map<int, vector<Vehicle>> &predictions,
                        map<string, float> &data)
{
    double cost = 0;
    //if (trajectory[1].state.compare("KL") == 0) return 0;
    if (trajectory[1].state.compare("PLCR") == 0) cost = 0.5;
    if (trajectory[1].state.compare("PLCL") == 0) cost = 0.5;
    if (trajectory[1].state.compare("LCL") == 0) cost = 1;
    if (trajectory[1].state.compare("LCR") == 0) cost = 1;
    //std::cout << "Stability " << trajectory[1].state << " " << cost << std::endl;
    return cost;
}

map<string, float> get_helper_data(const Vehicle & vehicle,
                                    const vector<Vehicle> & trajectory,
                                    const map<int, vector<Vehicle>> &predictions)
{
    map<string, float> trajectory_data;
    Vehicle trajectory_last = trajectory[1];
    float intended_lane;

    if (trajectory_last.state.compare("PLCL") == 0)
    {
        intended_lane = trajectory_last.lane + 1;
    } else if (trajectory_last.state.compare("PLCR") == 0)
    {
        intended_lane = trajectory_last.lane - 1;
    } else 
    {
        intended_lane = trajectory_last.lane;
    }

    float distance_to_goal = vehicle.goal_s - trajectory_last.s;
    float final_lane = trajectory_last.lane;

    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;

    return trajectory_data;
}

float calculate_cost(const Vehicle & vehicle, 
                    const map<int, vector<Vehicle>> & predictions,
                    const vector<Vehicle> & trajectory)
{
    map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    float cost = 0.0;

    vector<std::function<float(const Vehicle &, const vector<Vehicle> &, const map<int, vector<Vehicle>> &, 
    map<string, float> &)>> cf_list = {goal_distance_cost, inefficiency_cost, advance_cost, stability_cost};

    vector<double> weight_list = {REACH_GOAL, EFFICIENCY, ADVANCE, STABILITY};

    for (int i = 0; i < cf_list.size(); ++i)
    {
        float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
        cost += new_cost;
    }
    return cost;
}
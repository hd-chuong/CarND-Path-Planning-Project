#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "cost.h"
#include <iostream>
#include <math.h>
//#include "helpers.h"
using std::string;
using std::vector;

// too much buffer (40) leads to other car fill in in front of ego
// too narrow buffer hard to turn

const double BUFFER = 30; 
const double MAX_SPEED = 21.5;
const double MAX_ACCEL = 9;
const double GOAL_S = 1000000.0;
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s, float v, float a, string state)
{
    this->lane = lane;
    this->d = lane * 4 + 2;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    this->max_acceleration = MAX_ACCEL;
    this->LANE_SPEEDS = {MAX_SPEED,MAX_SPEED,MAX_SPEED};
    this->target_speed = this->LANE_SPEEDS[lane];
    this->previous_lane = this->lane;
    this->preferred_buffer = BUFFER; 
    this->goal_s = GOAL_S;
}

Vehicle::Vehicle(const Vehicle & v)
{
    this->lane = v.lane;
    this->d = v.d;
    this->s = v.s;
    this->v = v.v;
    this->a = v.a;
    this->state = v.state;
    this->LANE_SPEEDS = v.LANE_SPEEDS;
    this->max_acceleration = v.max_acceleration;
    this->target_speed = v.target_speed;
    this->previous_lane = v.previous_lane;
    this->yaw = v.yaw;
    this->preferred_buffer = BUFFER;
    this->goal_s = v.goal_s;
}

Vehicle& Vehicle::operator=( const Vehicle& v ) {
    this->lane = v.lane;
    this->d = v.d;
    this->s = v.s;
    this->v = v.v;
    this->a = v.a;
    this->state = v.state;
    this->max_acceleration = v.max_acceleration;
    this->target_speed = v.target_speed;
    this->previous_lane = v.previous_lane;
    this->yaw = v.yaw;
    this->preferred_buffer = BUFFER;
    this->LANE_SPEEDS = v.LANE_SPEEDS;
    this->goal_s = v.goal_s;
    return *this;
}

Vehicle::~Vehicle() {}

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> & predictions)
{
    //std::cout << "start choose next state " << std::endl;
    vector<string> states = successor_states();
    double cost;
    vector<double> costs;
    vector<vector<Vehicle>> final_trajectories;

    for (auto it = states.begin(); it != states.end(); ++it)
    {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);

        if (trajectory.size() != 0)
        {
            cost = calculate_cost(*this, predictions, trajectory);
            
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
        else {
            final_trajectories.push_back(trajectory);
            costs.push_back(1e8);
        }
        //std::cout << "State: " << *it << ". cost " << costs.back() << std::endl;
    }

    double best_idx = -1;
    double min_cost = 1e9;
    for (int i = 0; i < costs.size(); ++i)
    {
        if (min_cost > costs[i])
        {
            best_idx = i;
            min_cost = costs[i];
        }
    }

    //vector<float>::iterator best_cost_iter = min_element(begin(costs), end(costs));
    //best_idx = distance(begin(costs), best_cost_iter);

    //best_idx = 0; //always keep lane

    //std::cout << "keep_lane. ";
    //vector<Vehicle> result = keep_lane_trajectory(predictions);
    // std::cout << "Next lane: " << result[0].lane << std::endl;
    //return result;
    

    std::cout << "best idx " << best_idx;
    std::cout << " cost " << costs[best_idx];
    std::cout << " state " << states[best_idx] << std::endl;  
    //Vehicle next_state = final_trajectories[best_idx][1];
    
    //std::cout << "best behavior: " << states[best_idx] << ". Current state: " << this->state << ". Next lane " << next_state.lane << std::endl;
    // std::cout << "end choose next state " << std::endl;
    //std::cout << "Current Pos: " << this->s << ". Vel:" << this->v << ". Accel: " << this->a << std::endl;
    //std::cout << "Current Pos: " << next_state.s << ". Vel:" << next_state.v << ". Accel: " << next_state.a << std::endl;
    //return keep_lane_trajectory(predictions);
    //this->state = states[best_idx];
    return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() 
{
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    
    if (state.compare("KL") == 0)
    {
        if (lane < lanes_available - 1)
            states.push_back("PLCR");
            //states.push_back("LCR");
        if (lane > 0)
            states.push_back("PLCL");
            //states.push_back("LCL");
    }
    else if (state.compare("PLCL") == 0)
    {
        //states.pop_back();
        states.push_back("PLCL");
        states.push_back("LCL");
    }
    else if (state.compare("PLCR") == 0)
    {
        //state.pop_back();
        states.push_back("PLCR");
        states.push_back("LCR");
    }
    else if (state.compare("LCL") == 0)
    {
        if (this->d - this->previous_lane * 4 - 2 >= 2)
        {
            std::cout << "Previous lane " << this->previous_lane << std::endl;
            // states.pop_back();
            states.push_back("LCL");
            //states.push_back("PLCL");
        }
    }
    else if (state.compare("LCR") == 0)
    {
         if (this->d - this->previous_lane * 4 - 2 <=  -2)
         {
             std::cout << "Previous lane "<< this->previous_lane << std::endl;
            // states.pop_back();
            states.push_back("LCR");
            //states.push_back("PLCR");
        }
    }
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, 
                                      map<int, vector<Vehicle>> &predictions)
{
    vector<Vehicle> trajectory;

    if (state.compare("CS") == 0)
    {
        trajectory = constant_speed_trajectory();
    }
    else if (state.compare("KL") == 0)
    {
        trajectory = keep_lane_trajectory(predictions);
    } 
    else if (state.compare("LCL") == 0 || state.compare("LCR") == 0)
    {
        trajectory = lane_change_trajectory(state, predictions);
    }
    else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0)
    {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }
    return trajectory;
}

/*
 Gets next timestep kinematics (position, velocity, acceleration)
 for a given lane. Tries to choose the maximum velocity and acceleration given other vehicle positions and accel/velocity constraints.
*/
vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> &predictions, int new_lane)
{
    //std::cout << "For lane " << lane << ". ";
//    float max_velocity_accel_limit = this->max_acceleration + this->v;
    float new_position, new_velocity, new_accel;
//    float max_velocity_in_front = 100000000.0;
    Vehicle vehicle_ahead, vehicle_behind;

    double target_v;
    //std::cout << "buffer " << this->preferred_buffer << std::endl; 
    if (get_vehicle_ahead(predictions, new_lane, vehicle_ahead))
    {
        //std::cout <<"vehicle ahead !! Lane " << new_lane << "other vec " << vehicle_ahead.v << std::endl;
        //std::cout << "!!!! Vehicle ahead!" << std::endl;
        if (fabs(vehicle_ahead.s - vehicle_ahead.v - this->s) <= 40) 
        target_v = std::max(vehicle_ahead.v - 20.0, 0.5);
        // if (get_vehicle_behind(predictions, lane, vehicle_behind))
        // {
        //     target_v = vehicle_ahead.v;
        //     //new_velocity -= delta_sigmoid(this->v - target_v) * this->max_acceleration;
        // } 
        // else
        // {
        //     //float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * this->a;
            
        //     //target_v = std::min(std::min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
        //     target_v = this->target_speed;
        // }
    } 
    else 
    {
            target_v = this->LANE_SPEEDS[new_lane];
          //target_v = this->target_speed;
    }
    //std::cout << "velocity in front: " << max_velocity_in_front << "max vel accel limit" << max_velocity_accel_limit << ". target_speed " << target_speed << std::endl; 
    // std::cout << "Target v: " << target_v << ". This velocity " << this->v << std::endl;
    // new_velocity = this->v + delta_sigmoid(target_v - this->v) * this->max_acceleration;

    new_velocity = target_v;
    new_accel = new_velocity - this->v;
    new_position = this->s + new_velocity + new_accel / 2.0;

    //std::cout << "ego: " << "new s = " << new_position << " new v = " << new_velocity << std::endl;
    return {new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    // Generate a constant speed trajectory
    float next_pos = position_at(1);
    vector<Vehicle> trajectory = {
        Vehicle(this->lane, this->s, this-> v, this->a, this->state), 
        Vehicle(this->lane, next_pos, this->v, 0, this->state)};
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> & predictions)
{

    vector<Vehicle> trajectory = {
        Vehicle(this->lane, this->s, this->v, this->a, this->state)
    };

    vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    //std::cout << "new s: " << new_s << ". new v: " << new_v << ". new a: " << new_a << std::endl;   
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle> > & predictions)
{

    // ideal changing state kinematics

    float new_s, new_v, new_a;
    Vehicle vehicle_behind;

    int new_lane = this->lane + lane_direction[state];

    vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

    vector<Vehicle> trajectory;

    if (new_lane > 2 || new_lane < 0) return trajectory;

    trajectory.push_back(Vehicle(
        lane, this->s, this->v, this->a, this->state
        ));
    // if (get_vehicle_behind(predictions, this->lane, vehicle_behind))
    // {
    //     // Keep speed of current lane so as not to collide with car behind
    //     new_s = curr_lane_new_kinematics[0];
    //     new_v = curr_lane_new_kinematics[1];
    //     new_a = curr_lane_new_kinematics[2];
    // } 
    // else 
    vector<float> best_kinematics = get_kinematics(predictions, new_lane);

    // Choose kinematics with highest velocity
    // std::cout << "At prep change " << "cur pred v = " << curr_lane_new_kinematics[1] << " . new v = " << next_lane_new_kinematics[1] << std::endl; 
    // if (next_lane_new_kinematics[1] > curr_lane_new_kinematics[1])
    // {
    //     best_kinematics = next_lane_new_kinematics;        
    //     //std::cout << " Should change lane to " << this->lane + lane_direction[state] << std::endl;
    // }
    // else
    // {
    //     best_kinematics = curr_lane_new_kinematics;
    //     //std::cout <<" Should keep lane. Possible lane " << this->lane + lane_direction[state] << std::endl;
    // }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
    
    //
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> & predictions)
{
    //std::cout << "Start lane change" << std::endl;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;

    if (new_lane > 2 || new_lane < 0) return trajectory;

    Vehicle next_lane_vehicle, this_lane_vehicle;

    // now
    for (auto it = predictions.begin(); it != predictions.end(); ++it)
    {
        this_lane_vehicle = it->second[0];
        if (this_lane_vehicle.lane == this->lane)
        {
            bool tooCloseNow = fabs(this_lane_vehicle.s - this->s - this_lane_vehicle.v) <= 15;
            if (tooCloseNow)
            {
                return trajectory;
            }
        }
    }

    // when for new lane
    for (auto it = predictions.begin(); it != predictions.end(); ++it)
    {
        next_lane_vehicle = it->second[0];
        //std::cout << "check predictions lane change" << std::endl;
        //std::cout << "id: " << it->first << ". "  << next_lane_vehicle.lane << " ego " << "new lane " << new_lane << "current lane" << this->lane << std::endl;
        // if (next_lane_vehicle.lane == new_lane || next_lane_vehicle.lane == this->lane)
        //     std::cout << "Predict s = " << next_lane_vehicle.s << " " << "Predict ego " << this->s + this->v << "new lane " << new_lane << "current lane " << this->lane << std::endl;
        if ((next_lane_vehicle.lane == new_lane)) //|| next_lane_vehicle.previous_lane == new_lane))
        {
            bool tooClose = fabs(next_lane_vehicle.s - this->s - this->v) <= 20;
            bool egoOvertake = (this->s + this-> v >= next_lane_vehicle.s) && (next_lane_vehicle.s - next_lane_vehicle.v >= this->s);
            bool otherOvertake = (this->s + this-> v <= next_lane_vehicle.s) && (next_lane_vehicle.s - next_lane_vehicle.v <= this->s);
            // return empty trajectory
            if (tooClose || egoOvertake || otherOvertake)
                return trajectory;
        }
        //std::cout << "end predictions lane change" << std::endl;
    }
    // trajectory.push_back(Vehicle(
    //                     this->lane, 
    //                     this->s,
    //                     this->v,
    //                     this->a,
    //                     this->state));

    vector<float> kinematics = get_kinematics(predictions, new_lane);
    //std::cout << "Lane change v: " << kinematics[1] << ". Old v = " << this->v << std::endl;
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    //std::cout << "End total lane change" << std::endl;
    return trajectory;
}


void Vehicle::increment(int dt = 1)
{
    this->s = position_at(dt);
}

float Vehicle::position_at(int t)
{
    double s = this->s;
    double v = this->v;
    double a = this->a;
    //std::cout << "Position here s = " << s << " v = " << v << " a = " << a << std::endl;
    return s + v * t + a * t * t / 2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> & predictions, int lane, Vehicle & rVehicle)
{
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    double next_s = this->v + this->s;
    for (map<int, vector<Vehicle> >::iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == lane 
        && temp_vehicle.s <= next_s
        && temp_vehicle.s >= next_s - this->preferred_buffer
        && temp_vehicle.s > max_s)
        {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle> > & predictions, int lane, Vehicle & rVehicle)
{
    int min_s = this->goal_s;
    bool found_vehicle = false;

    Vehicle temp_vehicle;
    //std::cout << "This s = " << this->s << ". This d = " << this->d << std::endl;
    
    double next_s = this->s+ this->v;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        temp_vehicle = it->second[0];
        //std::cout << "next s = " << next_s << ". cur v = " << this->v << " ** lane " << lane << " other s = " << temp_vehicle.s << ". other v = " << temp_vehicle.v << "other lane " << temp_vehicle.lane << std::endl;
        if ((temp_vehicle.lane == lane) && (temp_vehicle.s < min_s)) 
        {
            double temp_cur_s = temp_vehicle.s - temp_vehicle.v;
            bool futureTooClose = (temp_vehicle.s >= next_s) && (temp_vehicle.s <= next_s + this->preferred_buffer) ;
            bool nowTooClose = (temp_cur_s >= this->s) && (temp_cur_s <= this->s + this->preferred_buffer);
            //bool egoSurpass = (next_s >= temp_vehicle.s) && (this->s <= temp_vehicle.s - temp_vehicle.v);
            //bool otherSurpass = (next_s <= temp_vehicle.s) && (this->s >= temp_vehicle.s - temp_vehicle.v);
            if (futureTooClose || nowTooClose)
            {
                if (temp_vehicle.s < min_s)
                {
                    min_s = temp_vehicle.s;
                    rVehicle = temp_vehicle;
                    found_vehicle = true;
                }
            }
        }
    }

    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon)
{
    //std::cout << "s = " << this->s << ". v = " << this->v << std::endl;
    // Generates predictions for non-ego vehicles to be used in trajectory generator for the ego vehicle.
    vector<Vehicle> predictions;
    // for (int i = 0; i < horizon; ++i)
    // {
    //     float next_s = position_at(i);
    //     float next_v = this->v;
    //     if (i < horizon-1)
    //     {
    //         next_v = position_at(i+1) - position_at(i);
    //     }
    //     //std::cout << "next_s = " << next_s << ". next_v = " << next_v << std::endl;
    //     predictions.push_back(Vehicle(this->lane, next_s, next_v, 0, "CS"));
    // }
    predictions.push_back(Vehicle(this->lane, position_at(1), this->v, 0, "CS"));
    return predictions;
}

void Vehicle::realize_next_state(vector<Vehicle> & trajectory)
{
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->v =   next_state.v;
    this->a = next_state.a;
}

void Vehicle::configure(vector<double> & road_data)
{
    // Called by simulator before simulation begins. Sets various parameters which will impact the ego vehicle.
    target_speed = road_data[0];
    lanes_available = 3;
    goal_s = road_data[1];
    max_acceleration = road_data[2];
}

void Vehicle::update(double x, double y, double s, double d, double v, double yaw, string state) {
    this->x = x;
    this->y = y;
    this->a = 0;
    this->s = s;
    this->d = d;
    this->v = v;
    this->yaw = yaw;
    //std::cout << "update in vechivle " << this->v << std::endl; 
    this->preferred_buffer = BUFFER;//this->v > 20 ? this->v : 20;
    this->state = state;
    this->max_acceleration = MAX_ACCEL;
    //this->preferred_buffer = 50;
    //std::cout << this->state << std::endl;
    // if (this->state.compare("KL") == 0 || this->state.compare("PLCL") == 0 || this->state.compare("PLCR") == 0)
    // {
    //     
    // }
    //if (this->state.compare("LCL") != 0 && this->state.compare("LCR") != 0)
    

    // if (this->state.compare("LCL") == 0 && this->d > 4 * this->previous_lane + 4)
    // {
    //     //this->previous_lane = this->lane + 1;
    // }
    // else if (this->state.compare("LCR") == 0 && this->d < 4 * this->previous_lane)
    // {
    //     //this->previous_lane = this->lane - 1;
    // }
    // else 
    //     this->previous_lane = this->lane;
    
    this->previous_lane = this->lane;
    //this->previous_lane = this->lane;
    this->lane = (int) d / 4.0;

    this->LANE_SPEEDS = {MAX_SPEED,MAX_SPEED,MAX_SPEED};
    this->target_speed = this->LANE_SPEEDS[this->lane];
    this->goal_s = GOAL_S;
    //std::cout << this->state.size() << std::endl;
}
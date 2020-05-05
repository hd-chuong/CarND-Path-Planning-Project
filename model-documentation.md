# Model description: Path Planning in Highway Driving Scenarios

[//]: # (Image References)

[fsm]: ./images/state-machine.png "State Machine"
[toplevel]: ./images/top-level.png "Top level architecture"
[sigmoid]: ./images/sigmoid-curve.png "Sigmoid function"
[function]: ./images/speed-change.png "Sigmoid function"

## Summary
This project aims to develop the autonomous stack for the car to drive in the highway scenario. This path planner was able to keep the car in lane, change the lanes and keep the speed with other cars around it. 

I decided to seperate the behavior planner entirely from the trajectory generator. In a high level, the behavior planner processes the sensor fusion data and return the most efficient and safe behavior. Receiving the future behavior, the trajectory generator will create the path which the car will exactly follow.

The screencast link: [Youtube](https://youtu.be/DcyLXSjdXWc)

![Top level architecture][toplevel]

## Behavior Planner
Compared to other settings, highway is relatively structured in the sense that all the driving signals, including lane lines, limit speeds are clearly defined. 

A finite state machine (FSM) is used to choose the best next behavior. The FSM consists of five states, which are summarized in the following table

![Finite State Machine][FSM]

|           State             | Abbreviation |              Description                                   |
|:---------------------------:|:------------:|:-----------------------------------------------------------|
| Keep Lane                   | KL           | Keep the car in the center of the current lane. if there is car ahead within the buffer distance, the ego car will keep its speed the same of the front car,  Otherwise, the ego car will accelerate to its maximum speed. |
| Prep left change            | PLCL         | Keep the car near the left mark of the current lane. The speed behavior is the same as the "Keep lane" behavior, i.e. keeping the front car's speed or reaching its maximum speed. |
| Prep right change           | PLCR         | Keep the car near the right mark of the current lane. The speed behavior is the same as the "Keep lane" behavior, i.e. keeping the front car's speed or reaching its maximum speed. |
| Left change                 | LCL          | Turn to the left lane, when it ensures that no car is too close to it, both in front and behind the ego car in the current lane and the new lane. |
| Right change                | LCR          | Turn to the right lane, when it ensures that no car is too close to it, both in front and behind the ego car in the current lane and the new lane. |

Each behavior will propose a 'trajectory', which contains two packages of data. Each package contains information about the lane, s position, velocity, acceleration and the state.

| State       |    First package     |      Second package      |
|:-----------:|:---------------------|:-------------------------|
| KL          | current lane, s, v, a, state | s, v, a, state in next 1 second and current lane |
| PLCL        | current lane, s, v, a, state | lane, s, v, a, state in next 1 second if the car is in a target left lane |
| PLCR        | current lane, s, v, a, state | lane, s, v, a, state in next 1 second if the car is in a target right lane |
| LCL         | lane, s, v, a in the next 1 second if the car is in the target left lane | lane, s, v, a in the next 1 second if the car is in the target left lane |
| LCR         | lane, s, v, a in the next 1 second if the car is in the target right lane | lane, s, v, a in the next 1 second if the car is in the target right lane |

For each current lane, there are a handful of next possible future states. Therefore, we introduce some cost functions which aim to collectively determine the best behavior for the future state. 
The cost functions include:
* Advance cost: car must advance in the highway setting. The behavior that stops the car or makes it travel backwards will be very highly penalized.
* Effciency cost: The behavior should choose the state whose speed is high so that the car could reach the goal quicker.
* Stability cost: state that requires the car to change lane has higher cost than the state that keeps the car stable. I.e, keep lane will have no cost, and "lane change left/right" have higher cost than "prepare left/right change".

As each cost has different influence on the behavior of the car. For example, the advance cost should be more influential than the efficient cost, because it is very dangerous to stop the car, or make it backwards in a highway, as a result, the advance cost should be given with a higher weight. The total cost will be the weighted sum of all individual costs.

Based on the costs, the behavior will the lowest cost will be selected as the next state. The behavior planner will send the data to the trajectory generator.

## Trajectory Generator       

This generator first determines the next s, d position and speed. The s and d position comes directly from the most desired data package. However, because the car acceleration is limited to less than 10 m/s^2, the speed change should be less than this max acceleration. Based on the suggested speed from the package, we use a sigmoid-like function to determine the actual speed. This idea of sigmoid-like function comes from the observation that the change in current speed should be relatively proportional to the different between target speed and current speed, but this speed change must not exceed maximum acceleration. Therefore, I used the sigmoid function because it has two levelling-off ends.
![Sigmoid curve][sigmoid]

![Sigmoid function][function]
Using the most desired d and s data, the trajectory generator will produce equally spaced waypoints in the s direction. To make sure that the transition in the path is smooth, the spline library is used. The points are first generated using the car coordinate system, then are translated and transformed back to the global coordinate system. 

## Future improvement
- Jerk minimization should be included in the trajectory generator to ensure that the driving experience is the most comfortable.
- In this trajectory generator implementation, the waypoints are equally spaced along the x direction, means that the acceleration could be instantaneously high at some points. Because of increasing or decreasing speeds, the waypoints should be modified to be more sparsely or more densely spaced.

# Highway Driving: Udacity Self Driving Car project
## James Singh, April 2019

In this project, I have built a path planner that safely navigates the simulated vehicle around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. It meets the below criteria required by the rubric: 

- The car is able to drive at least 4.32 miles without incident
- The car drives according to the speed limit. It doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.
- Max Acceleration and Jerk are not Exceeded. The car drives within a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
- Car does not have collisions.
- The car is able to change lanes
- The car stays in its lane, except for the time between changing lanes.

[![Highway Driving](https://github.com/jmsktm/CarND-Path-Planning-Project/blob/master/public/images/intro-thumbnail.png)](https://www.youtube.com/watch?v=jCQqaIxmlxs)

## Project/code structure
This is my very first attempt ever coding in C++. So, pretty sure my code is probably far from good. But I have made some effort to try out several things, and learn in the process.

1. I have organized the logic across multiple files, each doing it's own *thing* thereby promoting *Single Responsibility Principle*.
2. I have used [Catch2](https://github.com/catchorg/Catch2) for Unit Tests.
3. I have used [FakeIt](https://github.com/eranpeer/FakeIt) for mocking.

## Files
I have created .h files which aren't just headers. I wish I had more time to do it right.

### helpers.h
The provided helper file. No change.

### main.cpp
I have refactored main.cpp, and used the classes - *Telemetry*, *SmoothPlanner*, *Map* and *Vehicle*. It's much more organized now.

When the application is first run, it creates an instance of a vehicle (for ego vehicle) and the map. These are made available to the onMessage event handler.

![main.cpp](https://github.com/jmsktm/CarND-Path-Planning-Project/blob/master/public/images/main-classes.png)

### map.h
It stores the coordinates from the highway_map.csv file as a vector of waypoints.

### telemetry.h
It takes the telemetry message string passed in the websocket event, parses it, and makes the state available through behaviors such as *get_surrounding_vehicles()*, *_getTelemetryData()* etc. More importantly, it consists of the path planning function *get_smooth_curve()* which returns the next set of x- and y- coordinates for the vehicle to traverse.

Because of time constraints, I have defined the logics related to ***cost function*** and ***state transition*** in the same class for now.

### vehicle.h
As mentioned above, an instance of the *Ego vehicle* is created when the application is first started. After that, the state of the vehicle (the x and y coordinates, velocity components vx and vy, and the frenet coordinates s and d) are updated on every telemetry event. On that event, a map of vehicles close to the ego vehicle is also created from the node **sensor_fusion** of the telemetry data.

I have used these information to provide some behaviors on intra- and inter-vehicle dynamics (other than the getters for x, y, vx, vy, s and d) such as:

```
- get_speed(): double
- get_yaw(): double
- get_yaw_in_degrees(): double
- get_lane(): int
- get_ref_lane(): int
- prepare_lane_change_left(): void
- prepare_lane_change_right(): void
- print_vehicle_information(): void
- behind(Vehicle another_vehicle): bool
- same_lane_as(Vehicle another_vehicle): bool
- immediate_left_of(Vehicle another_vehicle): bool
- immediate_right_of(Vehicle another_vehicle): bool
- distance(Vehicle another_vehicle): double
- abs_distance(Vehicle another_vehicle): double
- closest_vehicle_ahead_distance(std::map<int, Vehicle> &vehicles): double
- time_to_complete_lane_change(): double
- closest_distance_during_lane_change(Vehicle &other_vehicle): double
- approaching(Vehicle &another_vehicle): bool
- vehicle_ahead_of_me(std::map<int, Vehicle> &vehicles): Vehicle
- keep_lane(std::map<int, Vehicle> &vehicles): void
```
Having these encapsulated has helped immensely in making the code more readable, as well as clarity of logic elsewhere in the project.

### planners/simpleplanner.h, planners/smoothplanner.h
I started off creating a simple planner (`planners/simpleplanner.h`) that would traverse straight between the waypoints at the max velocity. This is no longer used.

I replaced it with `smooothplanner.h` as a drop-in replacement for the planner. It's doesn't exactly fit into the `Liskov Substitution principle` of design pattern right now in the absence of a pure interface (lack of time again). But it kind of gives the sense. The SmoothPlanner comprises of a constructor that take objects of `Map` and `Telemetry`, and provides a behavior `getRoute()` which calls into Telemetry#getRoute() for the next set of x- and y- coordinates for the future path.


### props.h, config/config.json, config/config.cpp
I have moved the configurations out from the code to the json file `config/config.json`. The file `config/config.cpp` provides some logic to read data from the json file, that `props.h` uses to provide config to the caller.

### utils.h
Some utility static functions for purposes like printing out a message, set of coordinates etc.

### test/*_tests.cpp
Provides unit tests for implementation classes. Eg `test/map_tests.cpp` comprises of unit tests for the methods in map.h.


## Cost functions and State transition
While driving, the vehicle makes decision on whether to keep lane, switch lane to left or right, or accelerate/slow down based on the cost function.

Here's the diagram from Udacity which I have followed to perform state transition:

![State transition diagram by Udacity](https://github.com/jmsktm/CarND-Path-Planning-Project/blob/master/public/images/state_diagram.png)

The logic is arranged in four neatly organized methods in `telemetry.h`.

- cost\_left\_lane()
- cost\_right\_lane()
- cost\_current\_lane()
- process\_cost\_function()

![Cost functions](https://github.com/jmsktm/CarND-Path-Planning-Project/blob/master/public/images/cost_function.png)

#### cost\_left\_lane() / cost\_right\_lane()
- If the vehicle is already in the far lane or far right lane, the cost is returned as a big number (500 in my case).
- The cost for any lane switch alone is 50.
- If there are other vehicles in the proximity in the target lane, that will add to the cost. In my code, I am checking for how close other vehicles will come to the ego vehicle (considering the position and the velocity of both the vehicles) during the period of lane transition. The logic for that is contained in the method `Vehicle#closest_distance_during_lane_change(Vehicle &other_vehicle`. Based on this, and whether the two vehicles will be approaching or going far from each other after that, I am making some additional calculations for the cost. Eg. if the closest they'll get during the lane transition is below 10 meters, and they'll be approaching each other afterwards, I have marked it as a scenario of possible collision, and I'm returning the cost as 500 (same as that of leaving the road).

![Cost functions for switching to the left lane](https://github.com/jmsktm/CarND-Path-Planning-Project/blob/master/public/images/cost_left_lane.png)

#### cost\_current\_lane()
![Cost functions for keeping lane](https://github.com/jmsktm/CarND-Path-Planning-Project/blob/master/public/images/cost_current_lane.png)
I have a pretty simple implementation for that. If is no vehicle within 100 meters ahead of the ego vehicle in the same lane, the cost is zero. If there is, it's how many meters they are within the last 100 meters. Say, if the closest vehicle ahead of of the ego vehicle is at 30 meters, the cost is 100-30=70.

#### process\_cost\_function()
If the cost of switching lane to the left or right is less than 80, and is lesser than the cost of staying in the current lane, we prepare for lane change in that direction. 

![Prepare lane change left/right](https://github.com/jmsktm/CarND-Path-Planning-Project/blob/master/public/images/prepare_lane_change.png)

Preparing for lane change is just about setting the value for `ref_lane` to the left or right lane. Then in subsequent event calls, the route builder will consider the waypoints from the ref-lane for generating the path coordinates using spline.

If the lane change criteria is not met, the vehicle just follows the same lane, by adjusting distance and velocity with the car ahead of it. The definition for lane keeping is available in vehicle.h.

![Keep lane](https://github.com/jmsktm/CarND-Path-Planning-Project/blob/master/public/images/lane_keeping.png)

When the vehicle is running on the simulator, I am writing the cost of lane change/keeping to the console/log. It's formatted as:

```
30 <--- 50 ---> 40
```

Where,  
`30` is the cost of lane change to the left
`40` is the cost of lane change to the right, and
`50` is the cost of keeping current lane.

Here's the screenshot from Udacity's workspace showing that:
![Workspace log: Cost of lane change/keeping](https://github.com/jmsktm/CarND-Path-Planning-Project/blob/master/public/images/lane_change_screenshot.png)

### Issues
1. There are possibly tons of non-functional issues related to improving performance, caching, and using proper standards and design patterns to improve it. I have just not been able to optimize them further in the interest of time, and my limited knowledge of C++ as of now.
2. The lane keeping isn't that great right now. In the video I have include above, the vehicle navigates fine without collision for around 10 miles. But in many instances, our ego vehicle goes too close first, and then too far; sometimes leading to a collision. I could probably use reference_velocity better to maintain constant distance between the vehicles, or maybe the chapter (PID control) will be of help in this regards.
3. I have seen the vehicle go out of lane once or twice. That can be looked into further.

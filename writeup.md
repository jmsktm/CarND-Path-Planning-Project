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

![Highway Driving](https://github.com/jmsktm/CarND-Path-Planning-Project/blob/master/public/images/main-classes.png)

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


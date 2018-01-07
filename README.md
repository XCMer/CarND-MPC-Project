# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## State equations

### Shift in co-ordinate system

When we get way-points from the simulator, we convert it into the car's co-ordinate system for convenience. This is
the equation that we use:

```
point_car_x = (point_map_x - car_x) * cos(car_psi) + (point_map_y - car_y) * sin(car_psi);
point_car_y = -(point_map_x - car_x) * sin(car_psi) + (point_map_y - car_y) * cos(car_psi);
```

### Fitting the polynomial

Given the waypoint in the car's co-ordinate system, our polynomial starts at (0, 0) in the new-cordinate system. We fit
a 3rd order polynomial to the waypoints.

### Cross-track error

For the cross-track error, it's mathematically inefficient to calculate the shortest distance between the 3rd order
polynomial and the waypoint. Thus, we evaluate the distance between the waypoint and the car's y position at x=0 in
the car's co-ordinate system.

### Accounting for the delay

We assume a delay of 0.1s. We then use kinematic equations to get where the vehicle would be 0.1s from the current
time. All this is in the car's co-ordinate system.

```
x_t1 = = v_t * delay
y_t1 = 0
psi_t1 = -v_t * steer_value / Lf * delay
v_t1 = v_t + throttle_value * delay
cte_t1 = cte_t + v_t * sin(epsi_t) * delay
epsi_t1 = espi_t + psi_t1 // simplified on purpose
```

### State

Thus, the state becomes the values calculated above. By the time actuators take effect, the car would have moved to
state t+1 if it's currently in state t.

## Cost functions

Square errors were used for all the cost functions, since we were only interested in the magnitude of the error. One
additional benefit of the square error is that it penalizes larger errors more.

### epsi

I've used a weight of 5000 for the `epsi`. This was the first weight that I optimised. The rationale was, optimising
for CTE first was resulting in the car swiveling out of control, so it was important to establish a low error in
the direction.

### CTE

The CTE's weight is lower than `epsi` at 1000, and this was intentional. If I kept this greater then epsi, then the
car used to get sudden jerks to get the CTE under control.

### Cost on speed

In order to prevent the "stopping problem", wherein the car would come to halt at the minima of the cost function, I keep
a reference velocity of 40mph. The system is penalized if it does not maintain this speed.

### Cost of actuator usage - steering

The cost for using the steering has a weight of 10000. Or else, the car repeatedly used to oscillate along the waypoint. This
helped stabilize it, and made sure that it didn't go out of control during sharp turns.

### Cost of actuator usage - throttle

For the throttle, I put in a weight of 100 because the car used to intermittently break and accelerate. However, this is
lower than the steering weight, or else the car used to lose stability during turns.

### Change in actuator usage between timestamps

To prevent jerking of the car, I added a cost function to minimize the difference between actual values between
two consecutive timestamps. This further reduced all the oscillations, except for right before sharp turns.

## Equations for minimize cost of the planned path

In order to minimize the cost of the planned path (as defined above) with the waypoint, I used the kinematic equations again.
Here we minimize the error between the next point in the polynomial and the next point as predicted by the kinematic equations
from the current state.

```
constraint on x = x1 - (x0 + v0 * cos(psi0) * dt)
constraint on y = y1 - (y0 + v0 * sin(psi0) * dt)
constraint on psi = psi1 - (psi0 + v0 * delta0 / Lf * dt)
constraint on speed = v1 - (v0 + a0 * dt)
constraint on CTE = cte1 - ((f0 - y0) + (v0 * sin(epsi0) * dt))
constraint on epsi = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt)
```

## Choice of N and dt

Since the delay in actuation is about 0.1s, it was a given that dt had to be <= 0.1 so that we can actuate at a good
time-resolution. I kept it at 0.1 to get more flexibility with N.

When I tried with N=50 (after completing other aspects of the project), it tossed my car out of the road right after it
started. This could probably be because the system could not give out driving signals quickly enough (because of the heavy calculations)
, and the actuation started lasting for longer than required as a result.


---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

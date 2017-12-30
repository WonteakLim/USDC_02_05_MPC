# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

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

## Algorithm Description

#### Model
To predict vehicle's motion, we use a kinematic model. In this model, the inputs are a steering angle(delta) and acceleration (a). Based on these input, vehicle state (x, y, psi) is updated each step. The update equations are below.

$$$ x_{t+1}=x_t+v_t*cos(\psi_t)*dt $$$
$$$ y_{t+1}=y_t+v_t*sin(\psi_t)*dt $$$
$$$ \psi_{t+1}=\psi_t+(v_t/L_f)*\delta_t*dt $$$
$$$ v_{t+1}=v_t+a_t*dt $$$

#### Timestep Length and Elapsed Duration
Timestep length and elapsed duration are strong factors to determine the performance of MPC. As timestep length is shorter, we can consider more detailed motion. Also, the elapsed duration help the vehicle prepare the future path in advance. However, the longer the elapsed duration is, the more computation becomes. For this reason, it is important to select appropriate values of timestep and duration. In this project, we select 0.1 and 1 second for steptime and duration. These values were found by a trial-and-error method.

#### Polynomial Fitting and MPC Preprocessing
We can get the way point in a global coordinate (fixed frame coordinate). In this coordinate, we cannot represent all the patterns of way points as polynomial because polynomial is a function of x. In other word, we cannot express the straight way points toward y-axis using a polynomial function. To solve the problem, we use a vehicle frame coordinate. In this coordinate, x-axis is always driving direction.

All the way points are converted from the global coordinate (fixed frame coordiante) into the vehicle frame coordinate (moving frame coordinate). Based on these way points, we can model a continuous reference path.

Since the MPC needs the desired posiiton in each step, we extract these position on the reference path based on the speed and timestep length.


#### Model Predictive Control with Latency
To deal with latency, we estimate the future position after 100 miliseconds (latency). The position can be calculated by current states and a vehicle model (if the current position is (0,0,0)).

$$$ x_f=v_t*dt $$$
$$$ y_f=0 $$$
$$$ \psi_f=(v_t/L_f)*\delta*dt $$$
$$$ v_f=v_t + a*dt $$$
$$$ cte_f=cte_t + v_t*sin(\psi^{err}_t)*dt $$$
$$$ \psi^{err}_f=\psi^{err}_t+(v_t/L_f)*\delta_t*dt $$$

The etimated future position is used as the initial position of Model Predictive Control (MPC) to compensate latency.

#### Simulation
[![Video Label](http://img.youtube.com/vi/NZsq8eb0y4w/0.jpg)](https://youtu.be/NZsq8eb0y4w)
## Car Driving With Model Predictive Control (MPC)

[![steering a car with MPC](http://img.youtube.com/vi/C5UILYChPAc/0.jpg)](https://www.youtube.com/watch?v=C5UILYChPAc)

This is project 5 of Term 2 of Udacity Self-Driving Car Nanodegree. Program controls steering wheel, gas and brake to drive a car on the road. It uses model predictive control (MPC). This means that you have model of controlled object and you use this model to predict and optimize its future states to make your controlled object behave in desired way.

## Car model

 This MPC uses simple kinematic model

![Car model](https://github.com/parilo/CarND-MPC-Project/blob/master/mpc_model.png "Car model")

* x, y - car coordinates
* psi - car rotation angle
* v - velocity magnitude
* delta - steering wheel position (-25 .. 25 degrees)
* a - acceleration (positive values) or brake (negative values). possible values are -1 .. 1
* Lf - distance between the front of the vehicle and its center of gravity

Vector of (x, y, psi, v) is car state that changes through time and by influence of actuators (delta, a) that MPC applies to the model. So MPC optimize current and future actuators to keep states in desired values using following loss function:

```
Loss = cte^2 + (epsi)^2 + (v - v_desired)^2 + delta^2 +
       a^2 + 1000 * (delta - delta_prev)^2 + 100 * (a - a_prev)^2
```

* cte - cross track error
* epsi - error of car orientation. It is difference between current psi and desired psi in the current point
* (v - v_desired)^2 - we want our car to drive with desired velocity
* delta^2 and a^2 - we want not to use big actuators values without reason
* 1000 * (delta - delta_prev)^2 - we want to have smooth steering commands between iterations
* 100 * (a - a_prev)^2 - we want to have smooth gas/break commands between iterations

## Optimization

Optimizer tries to predict N future actuations (sequence of delta and a) with dt seconds between actuations for given state (x, y, v, psi). Also it computes future states for that actuations (green line on the video). For actual control we use first predicted actuators values in a sequence. So we need to choose N and dt according to our task properly. We need to balance between optimization time and accuracy. For driving a car in simulator I choose

* N = 12
* dt = 0.05 sec

Also I tried (N = 10, dt = 0.1), (N = 20, dt=0.05) and other values around that and found that choose values let car drive with about 70 Mph. Bigger amount of steps and smaller time steps lead to bigger than needed computation time while less amount of steps and bigger time steps make optimization not accurate enough.

## Preprocessing

We want our car to drive as fast as possible in the center of the traffic lane which is specified as number of points in global coordinate system. We need ability to calculate cross track error and epsi in every point on the road. So we need to approximate center of traffic lane between points. I translate traffic lane points into car coordinate system and approximate center of the lane with cubic polynomial (yellow line on the video).

## Car Latency

We have delay between moment we get actuators values and its application by the car. So we need to handle that delay. For that I use additional constraints for optimizer. I freeze actuators values during the delay to actuators values of previous step. So as we cannot apply actuators during the delay, so car will use previous actuator values. Delay used in this project is 100 ms.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

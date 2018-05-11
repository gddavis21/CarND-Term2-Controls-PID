# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## PID Control Project Overview

The project entails implementing a plug-in algorithm for a car driving simulator, where the objective is to control the steering angle and speed so that the car stays driving safely within the road (as opposed to driving off the road). The plug-in receives periodic messages with current speed, heading angle and distance from center of road, and responds with messages that specify steering angle and acceleration.

The plug-in is required to utilize 1 or more PID Control based algorithms for computing the steering angle and/or acceleration values.

## Explanation of PID Control

PID Control is a straightforward and very popular approach for implementing a control systems feedback loop. The idea of PID Control is to regularly receive error measurements from the control system, and respond with error correction values that drive the system toward zero error in an efficient and stable manner.

PID stands for "Proportional-Integral-Derivative" and represents the basic strategy of the algorithm. PID Control decomposes the input error into 3 component parts--the proportional, integral and derivative terms--and computes the correction as a weighted sum of these terms.
  * The proportional term is simply the current error measurement.
  * The integral term is an accumulation of the error over time.
  * The derivative term is the rate of change of the error.

The algorithm is parameterized by the 3 coefficients for the weighted sum (called PID gain parameters). Algorithm effectiveness is highly sensitive to these parameters. Appropriate parameter settings for a given control system are non-obvious, and may require a significant amount of manual and/or automated tuning.

How does a weighted sum of proportional, integral and derivative terms result in an effective error correction?
  * The primary correction comes from the proportional term. A (negative) scalar multiple of the current error directly "pushes" the error back toward zero. The responsiveness of the PID controller to error magnitude scales directly with proportional gain--larger gain makes the controller correct faster.
  * The problem with strictly proportional control is latency. In general, using enough proportional gain to adequately correct the system "overshoots" the error and results in oscillatory behavior--the controller is efficient but not stable. Achieving efficiency _and_ stability requires adding another term to the correction that damps or slows the proportional correction. This is the role of the derivative term--a scalar multiple of the rate of change (derivative) between the current error and the previous error is subtracted from the correction. An appropriate selection of derivative gain (for a given proportional gain) serves to offset the proportional correction just enough to make it stable.
  * Neither proportional nor derivative gain account for any kind of consistent bias in the system. This is what the integral term comes into play. A running total error estimate is kept by the PID controller, and a scalar multiple of this total error is added to the correction to counteract the effect of any bias in the system.

## Parameter Tuning

My process for PID parameter tuning was relatively simple and entirely manual. I started with zero derivative & integral gain, and a very small proportional gain. I gradually increased proportional and derivative gain, looking for just enough proportional gain to steer the car through the corners, and just enough derivative gain to keep the car from getting into too much oscillation.

After I was satisfied with the proportional and derivative gains, I added a very small integral gain to counteract the small steering bias in the system. I really didn't notice much effect from the integral gain, but left it at a small value.

## Implementation Notes

Far more challenging than parameter tuning was the issue of dealing with large fluctuations in the update rate from the simulator. I attempted to make my plug-in robust to the following update-rate related factors:
  * I found that the update rate varied greatly for different window sizes of the simulator. Smaller sizes updated much more frequently.
  * I found that the higher the update rate, the faster the car could be driven and remain in control (relatively free from oscillations).
  * I ended up keeping track of the update rate from my plug-in, and using an additional PID controller to scale the desired speed up or down, depending on the recent update rate. In this way I could even adjust speed when the update rate fluctuated during a session (which happened a lot).

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

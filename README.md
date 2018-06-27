# PID controller


[//]: # (Image References)

[image1]: result_movie.gif "Result"

![alt text][image1]



## About PID controll
A PID controller (proportional-integral-derivative controller) is a controll loop feedback mechanism widly used in industrial control systems and a variety of toher applications requiring continuously modulated control.  A PID controller continuously calculates an error value e(t) as the difference between a desired setpoint (SP) and a measured process variable (PV) and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively) which give the controller its name.

## CTE (Cross Track Error)
A cross track error is a distance of the vehicle from trajectory. In theory it’s best suited to control the car by steering in proportion to Cross Track Error(CTE).

## The effect of each P, I, D components
- P : The propotional portion of the controller tries to control the car toward the center line against the cross track error. `Kp` is mulitplied by the CTE to calculate the steering angle. This results in the car overshooting the reference trajectory, then changing course and oscillating. Increasing `Kp` will make the the vehicle to oscillate faster. I settled on a proportional factor of 0.15 for the PID controller.

- I : The integral portion corrects tystemic bias such as steering drift. The integral factor `Ki` is multiplied by sum of all the previous CTE. The steering drift is not an issue in the simulation, so `Ki` was set to small value 0.0001.

- D : The differential portion makes to counteract the proportional trend to overshoot the center line by smoothing the approach to it. It is calculated by multiplying the differential factor `Kd` by the derivative of CTE. The larger `Kd` will make the steerign angle decrese faster as it reaches the reference trajectory and I set it to 5.0.


## How the coefficients were tuned
The coefficients were determined by manual tuning. First, I tuned P parameter to make the car follow the road. It starts overshooting gradually, so I tuned D parameter to reduce overshooting. The I parameter can help to reduce bias of the car and the coefficient influence a lot so that the small value is set for it.

## Result

Here's a [link to my video result](https://youtu.be/NPQNzKVdx4w)



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
</br>
</br>

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)



# CarND-Controls-PID
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

## Goal for this project

The purpose of this project is to understand what a PID controller is and to let
such a controller drive a car in the simulator.

## PID controllers

The goal of a controller is to regulate processes as part of a control loop. The controller receives a setpoint request (in our experiment the desired position of the car in the road) and compares it to a measured feedback. The setpoint is where we want the car to be, and the feedback can be thought of as where the car really is.

A PID controller is a particular type of control loop that adjuststs its output using 3 different aspects of the feedback:
* **P** stands for _Proportional_: The output of the proportional factor is the product of gain (in our code **Kp**) and measured error ε (in our code **CTE**). The greater the error, the greater the response. Setting the proportional gain too high causes a controller to repeatedly overshoot the setpoint, leading to oscillation.
The problem with a proportional only controller is that when the error becomes too small, the output becomes too small, leading to a steady state despite with an error - offset.

* **I** stands for _Integral_: The integral part keeps track of all the error encountered so far and gets added (if the error is positive) or subtracted (if negative). This way we can react properly even when the proportional factor is too small to work properly. If the offset remains steady, the _integral error_ becomes bigger, leading to a stronger response from the controller.
The downside to the integral factor (in our code the gain is **Ki**) is that it might make the controller to overshoot past the target setpoint.

* **D** stands for _Derivative_: the derivative is looking at the rate of change of the error. The more error changes, the larger the derivative factor (in our code the gain is **Kp**) becomes. The effect of the derivative is to counteract the overshoot caused by P and I.

## Tuning the PID controller hypeparameters - manual
I decided to tune the hyperparameters manually in order to get a better understanding of the effects of each factor / gain.

1. First I set all gains to zero an reduced the throttle to _0.1_.
Of course this led the car to quickly drive off road

2. I then increased a bit the **Proportional** gain up to _0.2_. Interestingly, because the throttle is so small in value, the car is able to drive around the circuit, slowly but properly in the middle of the road as shown in the video below.
My understanding of the reason why oscillations and offset disappear is because curves cancel out the unavoidable error of the proportional controller.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/P2uKW0rGuqc/0.jpg)](https://youtu.be/P2uKW0rGuqc)

3. Next step is to set the throttle back to the suggested value _0.3_ and play with the **Integral** gain.
At this speed, oscillations are heavily present at the beginning when only the Proportional factor is used and the car goes offroad after a few seconds.

4. Because the Integral gain doesn't cancel out the oscillations, I increased the value until I saw that the car would behave more or less the same as if it was not included. The idea is that, because oscillations will be taken care of by the derivative factor, the Ingegral factor needs to make sure that we take care of the offset when present.

5. I started increasing the **Derivative** factor until the car stopped oscillating so much that it would go offroad after a few seconds.
I found several combinations of gains that would allow the car to drive properly when the throttle is fixed ad _0.3_
For example: 0.2, 0.0045, 2.0, or 1.5, 0.0035, 2.9, or 0.17, 0.0035, 3.5.

Check the video below for the results

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/lZ1jo3QvKMg/0.jpg)](https://youtu.be/lZ1jo3QvKMg)

## Adjusting the speed

It is common sense to slow down when we need to curve, so I played around with a simple formula that would increase the throttle when the car is driving straight, and decrease the throttle otherwise.

     throttle = (1 - fabs(steer_value)) * 0.25 + 0.25;

The result is visible in the video below.
The car ends up driving a bit faster and with more oscillations than without the throttle control, but still withing the limits of the road.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/mO4LZDpeN3c/0.jpg)](https://youtu.be/mO4LZDpeN3c)

## No Speed adjustement

The video below shows the result of the car being controlled by a PID with: 
* Proportional gain: 0.17
* Integral gain: 0.0035
* Derivative gain: 3.5

And with constant throttle 0.3

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/XpE5XZD3H4w/0.jpg)](https://youtu.be/XpE5XZD3H4w)


## Possible improvements.

1. Speed control: Instead of manually writing a formula based on common sense, it would probably be better to explore writing a PID controller for the speed.
2. It can be interesting what hyperparameters an automatic method for param-tuning would end up with. Again, I chose to do it manually in order to get a better insight of what role plays each gain in the PID controller.

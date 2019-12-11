### PID Implementation for steering angle control 
Cross Track Error (CTE) minimization with the 3 PID hyperparameters 

--------
#### Target of the project
The target of this project is to implement 2 controllers :
- one PID controller for the steering angle control to be sure that the vehicle stays always on the track,
- one P controller for the respect of the maximum speed to be sure that the vehicle doesn't exceed the maximum speed accepted.

--------
#### Hypermarameters Definition
- Kp : the target of this hyperparameter is to control proportionnaly the CTE : it's the standard proportional hyperparameter,
- Kd : the target of this hyperparameter is to control the CTE  for every cycle time (to delete the oscillation) : it's the derivative hyperparameter : he removes the oscillation and permit to converge on the goal without oscillation,
- Ki : the target of this hyperparameter is to control the sum of the CTE, which cause a systematic error (bias) : it's the integral hyperparameter : he removes the systematic error.

For the maximum speed limitation, only the Kp is useful, that why we speek about P Controller.

---------
#### Hyperparameters tuning
The method used for this project was the "manual Tuning".
The others method "TWIDDLE" and "SGD" haven't be used.

For this "manual method", the pipeline is :
- the first step is to configure the maximum speed with 5 MPH at the beginning to find the best hyperparameters,
- the second step is to increase gradually the Kp,
- when the first oscillation comes, the Kd Hyperparameters must be gradually increased to delete these oscillation,
- after a couple of cycle time, the Ki hyperparameters will be tuned to delete the systematic mistake (bias).

This pipeline will be reproduce with 10 MPH and 15 MPH to find the best hyperparameters.

The result of this pipeline are : steering_angle_pid.Init(0.15,0.003,1.0).

For the second controller (P Controller), we don't used this pipeline, only the Kp hyperparameters must be configured with 0.1.

For this controller, the Kd and the Ki are not useful.

The result is : speed_pid.Init(0.1,0.000,0.0).

-----------
#### Others comments
The maximum speed is configured with 20 MPH.

The safety criterion are fullfilled : the vehicule stays always on the track and stays always under 20 MPH (maximum speed acepted).

The confort criterion are fullfilled : in the sharp turm (-1 <steer_value or steer_value > 1 ), the steer_value is configured with +/- 0.5.
The manual method was here choosen.

This method is very long and not totally  optimal (groping solution), a second version of this software will be made later with the Twiddle and the SGD algoritm.

-----------
### Dependencies

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

--------------
### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

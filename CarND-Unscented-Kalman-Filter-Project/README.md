# Unscented Kalman Filter Project

[Eigen.tar.gz]: ./src/Eigen.tar.gz
[main.cpp]: ./src/main.cpp
[ukf.cpp]: ./src/ukf.cpp
[tools.cpp]: ./src/tools.cpp
[tools.h]: ./src/tools.h
[CMakeLists.txt]: ./CMakeLists.txt
[//]: # (Image References)
[image1]: ./img/UKF_with_dataset1.png "dataset 1"
[image2]: ./img/UKF_with_dataset2.png "dataset 2"

List of Files
---
* [ukf.cpp][ukf.cpp]: provides the UKF algorithm.
* [tools.cpp][tools.cpp]: provides the RMSE calculation function. (And storing the NIS value function, but it doesn't work well)

How to run
---
1. upzip the [Eigen.tar.gz][Eigen.tar.gz] file
2. run the [simulator](https://github.com/udacity/self-driving-car-sim/releases/)
3. select the project 1/2: EKF and UKF 
4. go to the file directory and execute
```
cd build && ./ExtendedKF
```
if you see 
#### Listening to port 4567
#### Connected!!!
then all is done.

# Project rubric

Compling
---
My OS is ubuntu 16.04 LTS. There was no error in compling. I did not change [CMakeLists.txt][CMakeLists.txt].


Accuracy
---
#### px, py, vx, vy output coordinates must have an RMSE <= [.09, .10, .40, .30] when using the file: "obj_pose-laser-radar-synthetic-input.txt", which is the same data file the simulator uses for Dataset 1.

* UKF with dataset 1
![alt text][image1]

* UKF with dataset 2
![alt text][image2]


Follows the Correct Algorithm
---
#### Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.
* The overall sensor fusion algorithm can be found in [ukf.cpp][ukf.cpp] (lines 15-469).

#### Your Kalman Filter algorithm handles the first measurements appropriately.
* The first measurement is handled in [ukf.cpp][ukf.cpp] (lines 97-128).

#### Your Kalman Filter algorithm first predicts then updates.
* The sequence can be found in [ukf.cpp][ukf.cpp] (lines 130-147).

#### Your Kalman Filter can handle radar and lidar measurements.
* Radar and lidar measurement are handled seperately in [FusionEKF.cpp][FusionEKF.cpp] (lines 138-147).


Additional (NIS values) 
---
#### Checking NIS values and tuning noise parameters.
* As suggested in the lecture, I wanted to check the NIS values to see if the noise parameters(`std_a_` and `std_yawdd_`) were set appropriately. For each UKF step, I tried to store the NIS value in a dat file. This attempt has been made as follows.
1. [ukf.cpp][ukf.cpp]: Calculates the NIS (lines 355, 466).
2. [main.cpp][main.cpp]: Store the NIS by using `StoreNIS` function (line 132).
3. [tools.h][tools.h]: `StoreNIS` function declaration (line 34).
4. [tools.cpp][tools.cpp]: `StoreNIS` function definition (lines 57-86).
Unfortunately, this attempt has been failed. And I cannot find a reason why it doesn't work as expected because of my lack of c++ skills. 

* The currently set noise parameter values have been set with reference to other studendt's work.
** https://github.com/jeremy-shannon/CarND-Unscented-Kalman-Filter-Project
** https://github.com/NikolasEnt/Unscented-Kalman-Filter
** https://github.com/darienmt/CarND-Unscented-Kalman-Filter-P2



# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, src/ukf.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/f437b8b0-f2d8-43b0-9662-72ac4e4029c1)
for instructions and the project rubric.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).


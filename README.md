[//]: # (Image References)

[driveExample]:   ./pictures/drivingExample.PNG "Example Drive"

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

---

## Write up

* Valid Trajectories
    * Algorithm generating valid trajectories is divided into 3 steps:
        * Finding proceeding and trailing car on each line and recording distance and speed
        * Calculating optimum line which could be safely reach
        * Calculating safe speed and determining if emergency braking is needed
    * The algorithm generates trajectories which allow long stretches without incident example below:
   ![Example Drive][driveExample]
   It is achieved by selecting the line with car furthest away and if all lines become blocked fastest line is selected. Line selection algorithm prefers right line when overtaking is not needed.
    * Speed is limited to speed limit or line speed proceeding car speed if overtaking is not possible
    * Line selection algorithm defines target speed and line and patch generation algorithm makes sure to limit acceleration and jerk in all axis to allowed values.
    * Line selection algorithm makes sure target speed is appropriate, it also asks Patch generation algorithm to do emergency braking when car in front is dangerously close.
    
* Path generation 
    * Patch generation takes input from 
 

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


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

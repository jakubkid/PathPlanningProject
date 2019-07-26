[//]: # (Image References)

[driveExample]:   ./pictures/drivingExample.PNG "Example Drive"

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

---

## Write up

* Valid Trajectories
    * Algorithm generating valid trajectories is divided into 5 steps:
        * Find proceeding and trailing car on each line
            Record speed and distance of trailing and leading car on each line
        * Calculate optimum line
            Select the line which we should be driving now it has to be safe to reach (other cars) if there are several best choices car will always select right line
        * Adapt speed to avoid collisions
            Select safe speed for driving, either maximum allowed speed or follow speed. This step also decides if emergency breaking is needed.
        * Calculate desired patch based on desired line
            Calculate desired patch which will reach desired line while minimizing sideway accelerations using spline fit.
        * Linearize spline following desired speed with regards to maximum jerk and acceleration
            Select the points 0.02s apart which will reach desired speed with regards to maximum or comfortable (depending if emergency breaking is engaged) acceleration and jerk.
    * The algorithm generates trajectories which allow long stretches without incident example below:
   ![Example Drive][driveExample]
     
* Path generation 
    * Patch generation takes last 2 points form previous trajectory or current car position and based on desired line and speed and maximum allowed acceleration and jerk it generates points spaced 0.02s apart.
        First, 3 points are added spaced 30m apart steering car to desired line but moving maximum 0.5 line width at time to make line change smooth 
        Next, spline fit is calculated on all points.
        Finally, points spaced 0.02s are selected on the spline which will reach desired speed respecting maximum allowed jerk or acceleration.
 

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

[ukf_1]: img/ukf_1.png
[ukf_2]: img/ukf_2.png

# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

---

## Summary

This repository contains a C++ implementation of an Unscented Kalman Filter. It has been implemented as part of the Udacity Self-Driving Car Engineer Nanodegree Program.

## Data

There are three data files in the repository:

* 'obj_pose-laser-radar-synthetic-input.txt'
* 'sample-laser-radar-measurement-data-1.txt'
* 'sample-laser-radar-measurement-data-2.txt'

Each line in the data file represents either a lidar or radar measurement marked by "L" or "R" on the starting line. The next columns are either the two lidar position measurements (x,y) or the three radar position measurements (rho, phi, rho_dot). Then comes the time stamp and finally the ground truth values for x, y, vx, vy, yaw, yawrate.

Although the data set contains values for yaw and yawrate ground truth, there is no need to use these values. main.cpp does not use these values, and it only calculates RMSE for x, y vx and vy. 

Yaw and yawrate were included in the data file in case you want to go beyond the requirements and analyze the data in more depth.

## Evaluation

#### Data file: obj_pose-laser-radar-synthetic-input.txt

Accuracy with UKF - RMSE:

* 0.0679275
* 0.08268
* 0.330714
* 0.231399

Comparison of estimation and ground truth

![ukf_1.png][ukf_1]

NIS for both radar and lidar

![ukf_2.png][ukf_2]

#### Data file: sample-laser-radar-measurement-data-1.txt

Accuracy with UKF - RMSE:

* 0.0818539
* 0.0897234
* 0.6025
* 0.59177


#### Data file: sample-laser-radar-measurement-data-2.txt

Accuracy with UKF - RMSE:

* 0.195348
* 0.188252
* 0.404008
* 0.540661


## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt output.txt`

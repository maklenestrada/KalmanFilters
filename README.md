# Kalman Filters – C++ & MATLAB

## Overview  
This project implements Kalman Filters in **C++** and **MATLAB**. The Linear Kalman Filter in `KalmanFilter.h` is inspired by the [lecture video](https://www.youtube.com/watch?v=QNRmlgdN-eg) but restructured in my own style. I wrote the Extended Kalman Filter (EKF) as an extension. Work on the Unscented Kalman Filter (UKF) is in progress.

The MATLAB script `PendulumKF.m` was written by me and implements a Kalman Filter for a linearized inverted pendulum.

## Files

- **`main.cpp`**: C++ main program for a continuous-discrete Kalman Filter on an inverted pendulum.  
- **`KalmanFilter.h`**: Contains the Linear KF example and my EKF implementation in C++.  
- **`PendulumKF.m`**: MATLAB script for the inverted pendulum Kalman Filter

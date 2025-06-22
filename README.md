# Kalman Filters – C++ & MATLAB

## Overview  
This project contains implementations of Kalman Filters in **C++** and **MATLAB**, starting with the continuous-discrete Kalman Filter. The `KalmanFilter.h` file includes the example Linear Kalman Filter from the [lecture video](https://www.youtube.com/watch?v=QNRmlgdN-eg), rewritten in my own notation.

I wrote the Extended Kalman Filter (EKF) implementation myself as an extension to this base. Work is ongoing to add the Unscented Kalman Filter (UKF).

The MATLAB script `PendulumKF.m` was written by me and implements a Kalman Filter for a linearized inverted pendulum.

## Files

- **`main.cpp`**: C++ main program for continuous-discrete Kalman Filter.  
- **`KalmanFilter.h`**: Header file containing the example Linear KF from the video and my EKF implementation in C++.  
- **`PendulumKF.m`**: MATLAB script implementing a Kalman Filter for a linearized inverted pendulum system.

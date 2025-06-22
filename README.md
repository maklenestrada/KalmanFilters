# Kalman Filters – C++ & MATLAB

## Overview  
This project contains implementations of Kalman Filters in **C++** and **MATLAB**, starting with the continuous-discrete Kalman Filter. Work is ongoing to extend this to the Extended Kalman Filter (EKF) and Unscented Kalman Filter (UKF).

The C++ files (`main.cpp`, `KalmanFilter.h`) are based on a [Kalman Filter lecture](https://www.youtube.com/watch?v=QNRmlgdN-eg) and rewritten in my own notation. The MATLAB script `PendulumKF.m` was written by me and implements a Kalman Filter for a linearized inverted pendulum.

## Files

- **`main.cpp`**: C++ main program for continuous-discrete Kalman Filter.
- **`KalmanFilter.h`**: Header file implementing the core Kalman Filter logic in C++.
- **`PendulumKF.m`**: MATLAB script implementing a Kalman Filter for a linearized inverted pendulum system.

#include <iostream>
#include <Eigen/Dense>
#include "KalmanFilter.h"

using namespace std;

//// LKF Main
//int main()
//{
//    // Inverted Pendulum Constants (matching MATLAB setup)
//    constexpr double g = 9.81;    // gravity (m/s^2)
//    constexpr double m = 1.0;     // mass (kg)
//    constexpr double l = 0.5;     // length (m)
//
//    // Dimensions for state (x), input (u), and output (y)
//    constexpr int32_t x_dim{2};
//    constexpr int32_t u_dim{1};
//    constexpr int32_t y_dim{1};
//
//    // Instantiate Kalman Filter object
//    KF::KalmanFilter<x_dim, u_dim, y_dim> KF_Sim;
//
//    // Discrete-time system matrices (converted via ZOH, values from MATLAB)
//    Eigen::Matrix<double, x_dim, x_dim> A;
//    A << 0.903493481936636,  0.0967619292542672,
//            -1.89846905196872,   0.903493481936636;
//
//    Eigen::Matrix<double, x_dim, u_dim> B;
//    B << 0.0196751311036420,
//            0.387047717017069;
//
//    Eigen::Matrix<double, y_dim, x_dim> C;
//    C << 1.0, 0.0;
//
//    // Initial state estimate and covariance
//    Eigen::Vector<double, x_dim> X_est;
//    X_est << 0.1, 0.0;
//
//    Eigen::Matrix<double, x_dim, x_dim> Sigma;
//    Sigma << 0.02, 0.0,
//            0.0,  0.02;
//
//    // Measurement vector
//    Eigen::Vector<double, y_dim> Y;
//    Y << 0.11;
//
//    // Control input vector
//    Eigen::Vector<double, u_dim> U;
//    U << 0.0;
//
//    // Process noise covariance matrix
//    Eigen::Matrix<double, x_dim, x_dim> q;
//    q << 0.01, 0.0,
//            0.0,  0.01;
//
//    // Measurement noise covariance matrix
//    Eigen::Matrix<double, y_dim, y_dim> r;
//    r << 0.01;
//
//    // Run one Kalman Filter iteration: prediction + correction
//    KF_Sim.LKF(X_est, U, Y, Sigma, A, B, C, q, r);
//
//    // Output results
//    cout << "After Prediction Step:\n";
//    cout << "X = \n" << KF_Sim.Xp_vec() << "\n";
//    cout << "Sigma = \n" << KF_Sim.Sigmap_mat() << "\n\n";
//
//    cout << "After Correction Step:\n";
//    cout << "X = \n" << KF_Sim.Xp1_vec() << "\n";
//    cout << "Sigma = \n" << KF_Sim.Sigmap1_mat() << "\n\n";
//
//    return 0;
//}

// EKF Main
int main() {

    // Inverted Pendulum Constants (matching MATLAB setup)
    constexpr double g = 9.81;
    constexpr double m = 1.0;
    constexpr double l = 0.5;
    constexpr double dt = 0.1;

    // Dimensions for state (x), input (u), and output (y)
    constexpr int32_t x_dim{2};
    constexpr int32_t u_dim{1};
    constexpr int32_t y_dim{1};

    KF::KalmanFilter<x_dim, u_dim, y_dim> EKF_Sim;

    // Initial state estimate and covariance
    Eigen::Vector<double, x_dim> X_est;
    X_est << 0.1, 0.0;

    Eigen::Matrix<double, x_dim, x_dim> Sigma;
    Sigma << 0.02, 0.0,
            0.0, 0.02;

    // Control input (torque)
    Eigen::Vector<double, u_dim> U;
    U << 0.0;

    // Measurement vector (angle measurement)
    Eigen::Vector<double, y_dim> Y;
    Y << 0.11;

    // Process noise covariance
    Eigen::Matrix<double, x_dim, x_dim> Q;
    Q << 0.01, 0.0,
            0.0, 0.01;

    // Measurement noise covariance
    Eigen::Matrix<double, y_dim, y_dim> R;
    R << 0.01;

    // Nonlinear state transition function f(x,u)
    auto f = [dt, g, l](const Eigen::Vector<double, x_dim>& x, const Eigen::Vector<double, u_dim>& u) -> Eigen::Vector<double, x_dim> {
        Eigen::Vector<double, x_dim> x_next;
        double angle = x(0);
        double omega = x(1);
        double torque = u(0);

        double angle_next = angle + omega * dt;
        double omega_next = omega + ((-g / l) * sin(angle) + torque) * dt;

        x_next << angle_next, omega_next;
        return x_next;
    };

    // Jacobian of f with respect to state x at current estimate
    Eigen::Matrix<double, x_dim, x_dim> F;
    F << 1.0, dt,
            (-g / l) * cos(X_est(0)) * dt, 1.0;

    // Measurement function h(x) — angle only
    Eigen::Vector<double, y_dim> h;
    h << X_est(0);

    // Jacobian of measurement function H
    Eigen::Matrix<double, y_dim, x_dim> H;
    H << 1.0, 0.0;

    // Run EKF step
    EKF_Sim.EKF(X_est, U, Y, Sigma, f, F, H, h, Q, R);

    // Output results
    cout << "After Prediction Step (X_p):\n" << EKF_Sim.Xp_vec() << "\n\n";
    cout << "After Prediction Step (Sigma_p):\n" << EKF_Sim.Sigmap_mat() << "\n\n";
    cout << "After Correction Step (X_p1):\n" << EKF_Sim.Xp1_vec() << "\n\n";
    cout << "After Correction Step (Sigma_p1):\n" << EKF_Sim.Sigmap1_mat() << "\n\n";

    return 0;
}

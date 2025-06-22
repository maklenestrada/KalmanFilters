#ifndef KALMANFILTERS__KALMANFILTER_H
#define KALMANFILTERS__KALMANFILTER_H
#include <stdint.h>
#include <Eigen/Dense>

namespace KF {
    template<int32_t x_dim, int32_t y_dim>
    class KalmanFilter {
    public:
        KalmanFilter() = default;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Linear Kalman Filter

        // A = state transition matrix
        // q = process noise covariance
        void LKF_Prediction(const Eigen::Matrix<double, x_dim, x_dim> &A, 
                            const Eigen::Matrix<double, x_dim, x_dim> &q) 
        {
            X = A * X;
            Sigma = A * Sigma * A.transpose() + q;
        }

        // Y = measurement vector
        // C = measurement matrix
        // r = measurement noise covariance
        void LKF_Correction(const Eigen::Vector<double, y_dim> &Y, 
                            const Eigen::Matrix<double, y_dim, x_dim> &C,
                            const Eigen::Matrix<double, y_dim, y_dim> &r) 
        {
            const Eigen::Matrix<double, y_dim, y_dim> S = C * Sigma * C.transpose() + r;
            const Eigen::Matrix<double, x_dim, y_dim> K = Sigma * C.transpose() * S.inverse();
            const Eigen::Matrix<double, x_dim, x_dim> I = Eigen::Matrix<double, x_dim, x_dim>::Identity();

            X += K * (Y - C * X);
            Sigma = (I - K * C) * Sigma;
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Extended Kalman Filter

        // Prediction step:
        // f = nonlinear propagation f(x)
        // F = Jacobian of f at current estimate
        // Q = process noise covariance
        void EKF_Prediction(const Eigen::Vector<double, x_dim> &f, 
                            const Eigen::Matrix<double, x_dim, x_dim> &F, 
                            const Eigen::Matrix<double, x_dim, x_dim> &Q)
        {
            X = f;
            Sigma = F * Sigma * F.transpose() + Q;
        }

        // Correction step:
        // Y = actual measurement vector
        // h = predicted measurement vector h(x)
        // H = Jacobian of measurement function at current estimate
        // R = measurement noise covariance
        void EKF_Correction(const Eigen::Vector<double, y_dim> &Y,
                            const Eigen::Vector<double, y_dim> &h,  
                            const Eigen::Matrix<double, y_dim, x_dim> &H,
                            const Eigen::Matrix<double, y_dim, y_dim> &R)
        {
            const Eigen::Matrix<double, y_dim, y_dim> S = H * Sigma * H.transpose() + R;
            const Eigen::Matrix<double, x_dim, y_dim> K = Sigma * H.transpose() * S.inverse();
            const Eigen::Matrix<double, x_dim, x_dim> I = Eigen::Matrix<double, x_dim, x_dim>::Identity();

            X += K * (Y - h);
            Sigma = (I - K * H) * Sigma;
        }

        Eigen::Vector<double,x_dim>& Xvec() { return X; }
        const Eigen::Vector<double, x_dim>& Xvec() const { return X; }

        Eigen::Matrix<double,x_dim,x_dim>& Sigma_mat() { return Sigma; }
        const Eigen::Matrix<double, x_dim,x_dim>& Sigma_mat() const { return Sigma; }

    private:
        Eigen::Vector<double, x_dim> X;                   //State Vector
        Eigen::Matrix<double, x_dim, x_dim> Sigma;        //State Covariance Matrix
    };
}
#endif //KALMANFILTERS__KALMANFILTER_H

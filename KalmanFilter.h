#ifndef KALMANFILTERS__KALMANFILTER_H
#define KALMANFILTERS__KALMANFILTER_H
#include <stdint.h>
#include <Eigen/Dense>


namespace KF {
    template<int32_t x_dim, int32_t u_dim, int32_t y_dim>
    class KalmanFilter {
    public:
        KalmanFilter() = default;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Linear Kalman Filter
        void LKF(const Eigen::Vector<double, x_dim> &X_est,          // Prior state estimate
                 const Eigen::Vector<double, u_dim> &U,              // Control input
                 const Eigen::Vector<double, y_dim> &Y,              // Measurement vector
                 const Eigen::Matrix<double, x_dim, x_dim> &Sigma,   // Prior covariance
                 const Eigen::Matrix<double, x_dim, x_dim> &A,       // State transition matrix
                 const Eigen::Matrix<double, x_dim, u_dim> &B,       // Control input matrix
                 const Eigen::Matrix<double, y_dim, x_dim> &C,       // Measurement matrix
                 const Eigen::Matrix<double, x_dim, x_dim> &q,       // Process noise covariance
                 const Eigen::Matrix<double, y_dim, y_dim> &r)       // Measurement noise covariance
        {
            // Prediction step
            X_p = A * X_est + B * U;                                      // Predict state
            Sigma_p = A * Sigma * A.transpose() + q;                      // Predict covariance

            // Correction step
            const Eigen::Matrix<double, y_dim, y_dim> S =                 // Innovation covariance
                    C * Sigma_p * C.transpose() + r;

            const Eigen::Matrix<double, x_dim, y_dim> K =                 // Kalman gain
                    Sigma_p * C.transpose() * S.inverse();

            const Eigen::Matrix<double, x_dim, x_dim> I =                 // Identity matrix
                    Eigen::Matrix<double, x_dim, x_dim>::Identity();

            X_p1 = X_p + K * (Y - C * X_p);                               // Update state estimate
            Sigma_p1 = (I - K * C) * Sigma_p;                             // Update covariance
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Extended Kalman Filter
        void EKF(const Eigen::Vector<double, x_dim> &X_est,                // Prior state estimate
                 const Eigen::Vector<double, u_dim> &U,                    // Control input
                 const Eigen::Vector<double, y_dim> &Y,                    // Measurement vector
                 const Eigen::Matrix<double, x_dim, x_dim> &Sigma,         // Prior covariance
                 const std::function<Eigen::Vector<double, x_dim>(const Eigen::Vector<double, x_dim>&, const Eigen::Vector<double, u_dim>&)> &f,  // Nonlinear state function f(x,u)
                 const Eigen::Matrix<double, x_dim, x_dim> &F,             // Jacobian of f w.r.t. state
                 const Eigen::Matrix<double, y_dim, x_dim> &H,             // Jacobian of measurement function h w.r.t. state
                 const Eigen::Vector<double, y_dim> &h,                    // Predicted measurement h(x)
                 const Eigen::Matrix<double, x_dim, x_dim> &Q,             // Process noise covariance
                 const Eigen::Matrix<double, y_dim, y_dim> &R)             // Measurement noise covariance
        {
            // Prediction step
            X_p = f(X_est, U);                      // Nonlinear state prediction with control
            Sigma_p = F * Sigma * F.transpose() + Q; // Covariance prediction

            // Correction step
            const Eigen::Matrix<double, y_dim, y_dim> S = H * Sigma_p * H.transpose() + R;  // Innovation covariance
            const Eigen::Matrix<double, x_dim, y_dim> K = Sigma_p * H.transpose() * S.inverse();  // Kalman gain
            const Eigen::Matrix<double, x_dim, x_dim> I = Eigen::Matrix<double, x_dim, x_dim>::Identity();

            X_p1 = X_p + K * (Y - h);             // Updated state estimate
            Sigma_p1 = (I - K * H) * Sigma_p;     // Updated covariance
        }


        Eigen::Vector<double,x_dim>& Xp_vec() { return X_p; }
        const Eigen::Vector<double, x_dim>& Xp_vec() const { return X_p; }

        Eigen::Matrix<double,x_dim,x_dim>& Sigmap_mat() { return Sigma_p; }
        const Eigen::Matrix<double, x_dim,x_dim>& Sigmap_mat() const { return Sigma_p; }

        Eigen::Vector<double,x_dim>& Xp1_vec() { return X_p1; }
        const Eigen::Vector<double, x_dim>& Xp1_vec() const { return X_p1; }

        Eigen::Matrix<double,x_dim,x_dim>& Sigmap1_mat() { return Sigma_p1; }
        const Eigen::Matrix<double, x_dim,x_dim>& Sigmap1_mat() const { return Sigma_p1; }

    private:
        Eigen::Vector<double, x_dim> X_p;                   //Prediction State Vector
        Eigen::Matrix<double, x_dim, x_dim> Sigma_p;        //Prediction State Covariance Matrix
        Eigen::Vector<double, x_dim> X_p1;                  //Correction State Vector
        Eigen::Matrix<double, x_dim,x_dim> Sigma_p1;        //Correction State Covariance Matrix
    };
}

#endif //KALMANFILTERS__KALMANFILTER_H

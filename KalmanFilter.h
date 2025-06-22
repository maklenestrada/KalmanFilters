#ifndef KALMANFILTERS__KALMANFILTER_H
#define KALMANFILTERS__KALMANFILTER_H
#include <stdint.h>
#include <Eigen/Dense>

namespace KF {
    template<int32_t x_dim, int32_t y_dim>
    class KalmanFilter {
    public:
        KalmanFilter() = default;


        void Prediction(const Eigen::Matrix<float, x_dim, x_dim> &A, const Eigen::Matrix<float, x_dim, x_dim> &q) {
            X = A * X;
            Sigma = A * Sigma * A.transpose() + q;
        }

        void Correction(const Eigen::Vector<float, y_dim> Y, Eigen::Matrix<float, y_dim, x_dim> C,
                        Eigen::Matrix<float, y_dim, y_dim> r) {
            const Eigen::Matrix<float, y_dim, y_dim> temp{C * Sigma * C.transpose() + r};
            const Eigen::Matrix<float, x_dim, y_dim> K{ Sigma * C.transpose() * temp.inverse()};
            const Eigen::Matrix<float, x_dim, x_dim> I{Eigen::Matrix<float, x_dim, x_dim>::Identity()};

            X += K * (Y - C * X);
            Sigma = (I - K * C) * Sigma;

        }

        Eigen::Vector<float,x_dim>& Xvec() { return X; }
        const Eigen::Vector<float, x_dim>& Xvec() const { return X; }

        Eigen::Matrix<float,x_dim,x_dim>& Sigma_mat() { return Sigma; }
        const Eigen::Matrix<float, x_dim,x_dim>& Sigma_mat() const { return Sigma; }

    private:
        Eigen::Vector<float, x_dim> X;                   //State Vector
        Eigen::Matrix<float, x_dim, x_dim> Sigma;        //State Covariance Matrix
    };
}
#endif //KALMANFILTERS__KALMANFILTER_H

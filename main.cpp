//Followed Vid: https://www.youtube.com/watch?v=QNRmlgdN-eg


#include <iostream>
#include <Eigen/Dense>
#include "KalmanFilter.h"

using namespace std;
int main()
{
    constexpr int32_t x_dim{ 2 };
    constexpr int32_t y_dim{ 1 };

    KF::KalmanFilter<x_dim,y_dim> kalmanFilter;

    kalmanFilter.Xvec() << 0.0F ,  1.0F;
    kalmanFilter.Sigma_mat() << 1.0F , 0.0F,
                                0.0F , 1.0F;

    Eigen::Matrix<float,x_dim,x_dim> A;
    A << 1.0F , 1.0F,
         0.0F , 1.0F;

    Eigen::Matrix<float,x_dim,x_dim> q;
    q << 0.5F , 0.0F,
         0.0F , 0.5F;

    kalmanFilter.Prediction(A,q);
    cout << "After Prediction Step \n";
    cout << "X = \n" << kalmanFilter.Xvec() << "\n";
    cout << "Sigma = \n" << kalmanFilter.Sigma_mat() << "\n\n";

    Eigen::Matrix<float,y_dim,y_dim> r;
    r << 0.1F;

    Eigen::Vector<float, y_dim> Y;
    Y << 1.2F;

    Eigen::Matrix<float,y_dim,x_dim> C;
    C << 1.0F , 0.0F;


    kalmanFilter.Correction(Y,C,r);
    cout << "After Correction Step \n";
    cout << "X = \n" << kalmanFilter.Xvec() << "\n";
    cout << "Sigma = \n" << kalmanFilter.Sigma_mat() << "\n\n";

    return 0;

}

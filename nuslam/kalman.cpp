#include <nuslam/kalman.hpp>

namespace kalman
{

    filter::filter(arma::mat x1, arma::mat x2, arma::mat z1, arma::mat z2){
        A.eye(3,3);
        B.eye(3,3);
        Q.eye(3,3);
        R.eye(3,3);
        x_k = x1;
        x_k_prev = x2;
        z_k = z1;
        z_k_prev = z2;
    };

    filter::predict(void){
        x_k = A * x_k_prev;
        z_k = A * z_k_prev * A.t + Q;
    };

    filter::update(void){
        K = (z_k*H.t)*(H*z_k*H.t+R).i;
        x_p = x_k + K * (z_k - H*x_k);
        z_p = (I - K*H)*z_k;
    };

}
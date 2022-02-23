#include <kalman/kalman.hpp>
#include <armadillo>

namespace kalman
{

    filter::filter(arma::mat a, arma::mat b, arma::mat h, arma::mat q, arma::mat r){
        A = a;
        B = b;
        Q = q;
        R = r;
        H = h;
        I = arma::eye(3,3);
    };

    void filter::predict(arma::mat x_0, arma::mat S_0, arma::mat u){
        x_k = A * x_0 + B * u;
        arma::mat A_T = A.t();
        S_k = (A * S_0) * A_T + Q;

    };

    arma::mat filter::correct_x(arma::mat z_k){
        arma::mat H_T = H.t();
        K = S_k * H_T * (H*S_k*H_T+R).i();

        x_pred = x_k + K*(z_k - H*x_k);

        return x_pred; 
    };

    arma::mat filter::correct_S(){
        arma::mat H_T = H.t();
        K = S_k * H_T * (H*S_k*H_T+R).i();

        S_pred = (I - K*H) * S_k;

        return S_pred;
    };

}
#include <kalman/kalman.hpp>
#include <ros/console.h>
#include <armadillo>

namespace kalman
{

    filter::filter(arma::mat a, arma::mat b, arma::mat q){
        A = a;
        B = b;
        Q = q;
        I = arma::eye(3,3);
    };

    void filter::predict(arma::mat x_0, arma::mat S_0, arma::mat u){
        x_k = A * x_0 + B * u;

        ROS_WARN("x_k: ");
        x_k.print(std::cout);

        arma::mat A_T = A.t();

        ROS_WARN("A: ");
        A.print(std::cout);
        ROS_WARN("S_0: ");
        S_0.print(std::cout);
        ROS_WARN("A_T: ");
        A_T.print(std::cout);
        ROS_WARN("Q: ");
        Q.print(std::cout);

        arma::mat A_aug = arma::eye(5,5);
        arma::mat A_aug_T = A_aug.t();
        S_k = (A_aug * S_0) * A_aug_T + Q;

        ROS_WARN("S_k: ");
        S_k.print(std::cout);
    };

    arma::mat filter::correct_x(arma::mat h, arma::mat v_k){

        z_k = h + v_k;

        double r = z_k(0,0);
        double phi = z_k(1,0);
        double dx = r * cos(phi);
        double dy = r * sin(phi);
        double d = sqrt(pow(dx,2)+pow(dy,2));

        H.set_size(2,5);

        H(0,0) = 0;  
        H(0,1) = -dx/d; 
        H(0,2) = -dy/d; 
        H(0,3) = dx/d; 
        H(0,4) = dy/d; 

        H(1,0) = -1; 
        H(1,1) = dy/pow(d,2); 
        H(1,2) = -dx/pow(d,2); 
        H(1,3) = -dy/pow(d,2); 
        H(1,4) = dx/pow(d,2); 

        ROS_WARN("H: ");
        H.print(std::cout);

        arma::mat H_T = H.t();

        R.set_size(2,2);
        R(0,0) = v_k(0,0);
        R(0,1) = 0;
        R(1,0) = 0;
        R(1,1) = v_k(1,0);

        K = S_k * H_T * (H*S_k*H_T+R).i();

        ROS_WARN("K: ");
        K.print(std::cout);

        ROS_WARN("x_k: ");
        x_k.print(std::cout);

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
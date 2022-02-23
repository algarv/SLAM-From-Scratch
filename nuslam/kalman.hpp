#ifndef KALMAN_INCLUDE_GUARD_HPP
#define KALMAN_INCLUDE_GUARD_HPP

#include <armadillo>


namespace kalman
{

class filter
{
public:
    filter(arma::mat x1, arma::mat x2, arma::mat z1, arma::mat z2);

    arma::Mat<double> predict();

    arma::Mat<double> update();

private:
    arma::Mat<double> x_k(3,1,fill::zeros); //estimated state
    arma::Mat<double> z_k(2,1,fill::zeros); //measurement
    arma::Mat<double> x_p(3,1,fill::zeros); //predicted state
    arma::Mat<double> z_p(2,1,fill::zeros); //approximated / predicted measurement
    arma::mat A;
    arma::mat B; 
    arma::mat Q; 
    arma::mat R; 
    arma::mat x_k;
    arma::mat x_k_prev;
};

}

#endif
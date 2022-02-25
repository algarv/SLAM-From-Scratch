#ifndef KALMAN_INCLUDE_GUARD_HPP
#define KALMAN_INCLUDE_GUARD_HPP

#include <armadillo>


namespace kalman
{

/// \brief The framework for an extended Kalman filter estimating the two dimensional position of a differential drive robot
class filter
{
public:
    /// \brief Set-Up the frame work for an extended kalman filter
    /// \param a - Prediction Matrix
    /// \param b - Control Matrix
    /// \param q - State Noise Matrix
    /// \param r - Sensor Noise Matrix
    filter(arma::mat a, arma::mat b, arma::mat q);

    /// \brief Prediction step of the Kalman filter
    /// \param x_0 - current configuration
    /// \param S_0 - current covariance
    /// \param u - commanded velocity
    void predict(arma::mat x_0, arma::mat S_0, arma::mat u);

    /// \brief State correction step of the Kalman filter
    /// \param z_k - measured state
    arma::mat correct_x(arma::mat z_k, arma::mat v_k);

    /// \brief Covariance correction step of the Kalman filter
    arma::mat correct_S();

private:
    arma::mat x_0; //current state
    arma::mat x_k; //estimated state
    arma::mat x_pred; //predicted state

    arma::mat S_0; //covariance matrix of current state
    arma::mat S_k; //covariance matrix of estimated state
    arma::mat S_pred; //predicted covariance

    arma::mat z_k; //measured state
    arma::mat v_k; //sensor noise

    arma::mat A; //prediction matrix
    arma::mat B; //control matrix
    arma::mat Q; //noise covariance matrix
    arma::mat R; //sensor noise covariance matrix
    arma::mat H; //Observation matrix
    arma::mat K; //Kalman gain
    arma::mat I; //identity
    arma::mat h; //observation matrix
};

}
#endif
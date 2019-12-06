#pragma once

#include <armadillo>

class KalmanFilter
{
public:
  KalmanFilter(uint dim_in, uint dim_out, uint dim_state) : l(dim_in), m(dim_out), n(dim_state) {
    using arma::mat;
    using arma::vec;

    A = mat(n,n).eye();
    B = mat(n,l).zeros();
    C = mat(m,n).zeros();

    Q = mat(n,n).eye();
    R = mat(m,m).eye();
    P = mat(n,n).eye();

    K = mat(n,m).eye();
    I = arma::eye<mat>(n,n);

    u = vec(l).zeros();
    q_pred = vec(n).zeros();
    q_est = vec(n).zeros();
    y = vec(m).zeros();
  }

  void predictState() {
    q_pred = A * q_est + B * u;
    P = A * P * trans(A) + Q;
  }

  void correctState() {
    K = P * trans(C) * inv(C * P * trans(C) + R);
    q_est = q_pred + K * (y - C * q_pred);
    P = (I - K * C) * P;
  }

  void updateState() {
    predictState();
    correctState();
  }

public:
  // System matrices:
  arma::mat A;       // State
  arma::mat B;       // Input
  arma::mat C;       // Output

  // Covariance matrices:
  arma::mat Q;       // Process
  arma::mat R;       // Measurement
  arma::mat P;       // Estimate error

  // Kalman gain matrix:
  arma::mat K;

  // Identity matrix
  arma::mat I;

  // Signals:
  arma::vec u;       // Input
  arma::vec q_pred;  // Predicted state
  arma::vec q_est;   // Estimated state
  arma::vec y;       // Measurement

private:
  // Dimensions:
  uint l;             // Input
  uint m;             // Output
  uint n;             // State
};
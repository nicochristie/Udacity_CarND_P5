#include <iostream>
#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_; // (equation 101)
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;        // (equation 13)
  Update_x_and_P(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  // cartesian values
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  // polar values
  double rho = sqrt(px*px + py*py);
  double theta = atan2(py, px);
  double rho_dot = (px*vx + py*vy) / rho;
  
  // TODO: zero checks
  if (fabs(rho) < 0.0001) rho_dot = 0;
  
  // h(x') 
  VectorXd h = VectorXd(3);
  h << rho, theta, rho_dot; // (equation 53)
  
  VectorXd y = z - h;
  Update_x_and_P(y);
}

/**
 * Updates the x_ and P_ matrixes given an y value
 * Since the source of the measure (sensor) is
 * irrelevant we can use the same update method
 */
void KalmanFilter::Update_x_and_P(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_; // (equation 14)
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;      // (equation 15)
  
  // new estimate
  x_ = x_ + (K * y);              // (equation 16)
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;         // (equation 17)
}
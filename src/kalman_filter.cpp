#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in; /** < State Matrix */
  P_ = P_in; /** < Covariance matrix */
  F_ = F_in; /** State transiction matrix */
  H_ = H_in; /** Measurement matrix */
  R_ = R_in; /** Measurement covariance matrix */
  Q_ = Q_in; /** Process covariance matrix */
}

void KalmanFilter::Predict() {

  /*****************************************************************************
   *  Predict New State
  *****************************************************************************/
  
  x_ = F_ * x_; /** < Predict New State */
  MatrixXd Ft = F_.transpose(); /** Inverse of transition matrix F */
  P_ = F_ * P_ * Ft + Q_; /** < Predict Covariance State */

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * Update the state by using Kalman Filter equations
   */

  VectorXd z_predict = H_ * x_; /** < Predict True State */
  VectorXd y = z - z_predict;  /** < Error in prediction */
  MatrixXd Ht = H_.transpose(); /** < Transpose of State Measurement */
  MatrixXd S = H_ * P_ * Ht + R_;  /** < Innovation Covariance Matrix */
  MatrixXd Si = S.inverse(); /** < Inverse Innovation Covariance Matrix */
  MatrixXd PHt = P_ * Ht; /** Product of Estimate Covariance and State Measurement */
  MatrixXd K = PHt * Si;  /** < Optimal Kalman Gain */

  /*****************************************************************************
   *  New Estimate
  *****************************************************************************/
  
  x_ = x_ + (K * y); /** < Updated State Matrix */
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size); /** < Identity Matrix */
  P_ = (I - K * H_) * P_; /** < Updated Covariance Matrix */

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * Update the state by using Extended Kalman Filter equations
   */
  
  // Convert to Polar
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  const double threshold = 0.001; 
  if(fabs(px) < threshold && fabs(py) < threshold)
    px = py = threshold;
  
  /**
   *      h(x') = | ρ | =  |            sqrt(px^2 + py^2)           |
   *              | φ |    |           arctan(py'/px')              | 
   *              | ρ.|    |     px*vx + py*py / sqrt(px^2 + py^2)  | 
   * 
   *      Radial Distance (ρ), Bearing (φ), Radial Velocity (ρ.). 
   */
  
  double rho = sqrt(px*px + py*py);
  double phi = atan2(py, px);
  double rho_dot;

  if(fabs(rho) < threshold*threshold) /** Negligible Change */
    rho_dot = 0;
  else{
    rho_dot = (px*vx+py*vy) / rho;
  }

  VectorXd h_prime = VectorXd(3);
  h_prime << rho, phi , rho_dot;

  VectorXd y = z - h_prime; /** < Error in prediction */

  // Normalize phi to be between [-π , π]
  while(y(1) > M_PI)
    y(1) -= 2.0 * M_PI;
  while (y(1) < -M_PI)
    y(1) += 2.0 * M_PI;

  MatrixXd Ht = H_.transpose(); /** < Transpose of State Measurement */
  MatrixXd S = H_ * P_ * Ht + R_;  /** < Innovation Covariance Matrix */
  MatrixXd Si = S.inverse(); /** < Inverse Innovation Covariance Matrix */
  MatrixXd PHt = P_ * Ht; /** Product of Estimate Covariance and State Measurement */
  MatrixXd K = PHt * Si;  /** < Optimal Kalman Gain */

  /*****************************************************************************
   *  New Estimate
  *****************************************************************************/
  
  x_ = x_ + (K * y); /** < Updated State Matrix */
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size); /** < Identity Matrix */
  P_ = (I - K * H_) * P_; /** < Updated Covariance Matrix */

}

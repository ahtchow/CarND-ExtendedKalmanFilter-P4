#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {

 public:

  VectorXd x_; /** < State Vector */

  MatrixXd P_; /** < State Covariance Matrix */

  MatrixXd F_; /** < State Transition Matrix */

  MatrixXd Q_; /** < Process Covariance Matrix */

  MatrixXd H_; /** < Measurement Measurement Matrix */

  MatrixXd R_; /** < Measurement Covariance Matrix */

  /**
   * @brief Construct a new Kalman Filter object
   */
  KalmanFilter();

  /**
   * @brief Destroy the Kalman Filter object
   */
  virtual ~KalmanFilter();

  /**
   * @brief Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
            MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in);
  
  /**
   * @brief Prediction Predicts the state and the state covariance using the process model.
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * @brief Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const VectorXd &z);

  /**
   * @brief Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const VectorXd &z);

};
#endif // KALMAN_FILTER_H_

#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {

  is_initialized_ = false;
  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);  // Laser measurement covariance
  R_radar_ = MatrixXd(3, 3);  // Radar measurement covariance
  H_laser_ = MatrixXd(2, 4); // Measurement Matrix for Laser
  Hj_ = MatrixXd(3, 4); // Jacobian Matrix for Radar

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // Measurement Matrix -> Only takes account for position Px, Py
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1;

  // Initial covariance Matrix
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 10,  0,   0,   0,
             0,   10,  0,   0,
             0,   0,   100, 0,
             0,   0,   0,   100;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/

  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
    cout << "Extended Kalman Filter: " << endl;
    ekf_.x_ = VectorXd(4);
    cout << "  Sensor Type: " << measurement_pack.sensor_type_ << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      
      float rho = measurement_pack.raw_measurements_[0]; /** < RANGE: ρ (rho) */
      float phi = measurement_pack.raw_measurements_[1]; /** < BEARING: φ (phi) */
      float rho_dot = measurement_pack.raw_measurements_[2]; /** < RADIAL VELOCITY: ρ. (rho dot) */
      
      cout << "[ " << rho << " , " << phi << " , " << rho_dot << endl;
      
      // Normalize phi to be between [-π , π]
      while(phi > M_PI)
        phi -= 2.0 * M_PI;
      while (phi < -M_PI)
        phi += 2.0 * M_PI;

      // Convert to Cartesian 
      float x = rho * cos(phi);
      float y = rho * sin(phi);
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);

      ekf_.x_ << x, y, vx, vy;
    }
    
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      float x = measurement_pack.raw_measurements_[0];
      float y = measurement_pack.raw_measurements_[1];
      float vx = 0;
      float vy = 0;
      
      ekf_.x_ << x, y, vx, vy;

    }

    // Initial results
    cout << "EKF init: " << ekf_.x_ << endl;
    
    //Create state covariance matrix
    ekf_.P_ = MatrixXd(4,4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;
    
    //initial transition matrix
    ekf_.F_ = MatrixXd(4,4);
    ekf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;
    
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  /*****************************************************************************
   *  Prediction
  *****************************************************************************/

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;/** Δt, Change in time(s) */
  previous_timestamp_ = measurement_pack.timestamp_;

  /**
   * @brief Update State Transition matrix F
   * 
   *                   |   1   0   Δt  0   |  
   *              F  = |   0   1   0   Δt  |  
   *                   |   0   0   1   0   |  
   *                   |   0   0   0   1   | 
   */
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, dt,  0,
             0, 1,  0, dt,
             0, 0,  1,  0,
             0, 0,  0,  1;

  /**
   * @brief Update Process Noise Covariance Q
   * 
   *       Q = | Δt^4/4 * σ^2[ax]    0    Δt^3/2 * σ^2[ax]   0          |
   *           |     0    Δt^4/4 * σ^2[ay]     0       Δt^3/2 * σ^2[ay] |
   *           | Δt^3/2 * σ^2[ax]    0    Δt^2 * σ^2[ax]      0         |
   *           |     0    Δt^3/2 * σ^2[ay]     0       Δt^2 * σ^2[ay]   |
   * 
   */

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  float noise_ax = 9.0;
  float noise_ay = 9.0;

  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << dt_4/4*noise_ax, 0,               dt_3/2*noise_ax, 0,
             0,               dt_4/4*noise_ay, 0,               dt_3/2*noise_ay,
             dt_3/2*noise_ax, 0,               dt_2*noise_ax,   0,
             0,               dt_3/2*noise_ay, 0,               dt_2*noise_ay;


  ekf_.Predict(); /** < Predict next measurement. */

  /*****************************************************************************
   *  Update
  *****************************************************************************/

  /**
   *  Use the sensor type to perform the update step.
   *  Update the state and covariance matrices.
   *  Adjust Measurement Covariance Matrix and Measurement Matrix
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates:
    ekf_.R_ = R_radar_;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates: 
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;

}

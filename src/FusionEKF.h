#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class FusionEKF {
 public:

  /**
   * @brief Construct a new Fusion Extended Kalman Filter object.
   */
  FusionEKF();

  /**
   * @brief Destroy a Fusion Extended Kalman Filter object.
   */
  virtual ~FusionEKF();

  /**
   * @brief Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * @brief Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter ekf_; /** < ExteZnded Kalman Filter */

 private:

  bool is_initialized_; /** < Is initialized or not (first measurement). */

  long long previous_timestamp_; /** < Previous time stamp. */

  Tools tools; /** < Tool object used to compute Jacobian and RMSE. */

  MatrixXd R_laser_; /** < Laser Measurement Covariance Matrix. */

  MatrixXd R_radar_; /** < Radar Measurement Covariance Matrix. */

  MatrixXd H_laser_; /** < Laser State Conversion Matrix. */

  MatrixXd Hj_; /** < Jacobian State Conversion Matrix. */
  
};

#endif // FusionEKF_H_

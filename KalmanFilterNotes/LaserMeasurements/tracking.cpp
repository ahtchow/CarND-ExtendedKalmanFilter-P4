#include "tracking.h"
#include <iostream>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

Tracking::Tracking() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // create a 4D state vector, we don't know yet the values of the x state
  kf_.x_ = VectorXd(4);

  // state covariance matrix P
  kf_.P_ = MatrixXd(4, 4);
  kf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;


  // measurement covariance
  kf_.R_ = MatrixXd(2, 2);
  kf_.R_ << 0.0225, 0,
            0, 0.0225;

  // measurement matrix
  kf_.H_ = MatrixXd(2, 4);
  kf_.H_ << 1, 0, 0, 0,
            0, 1, 0, 0;

  // the initial transition matrix F_
  kf_.F_ = MatrixXd(4, 4);
  kf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

  // set the acceleration noise components
  noise_ax = 5;
  noise_ay = 5;
}

Tracking::~Tracking() {

}

// Process a single measurement
void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    //cout << "Kalman Filter Initialization " << endl;

    // set the state with the initial location and zero velocity
    kf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;

    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // TODO: YOUR CODE HERE
  // 1. Modify the F matrix so that the time is integrated
  kf_.F_(0,2) = dt; // Δt
  kf_.F_(1,3) = dt; // Δt
  // 2. Set the process covariance matrix Q
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  kf_.Q_ = MatrixXd(4,4);
  kf_.Q_ << dt_4/4 * noise_ax /* σ^2[ax] */ , 0  , dt_3/2 * noise_ax , 0,
            0 , dt_4/4* noise_ay /* σ^2[ay] */ , 0 , dt_3/2 * noise_ay,
            dt_3/2*noise_ax, 0 , dt_2 * noise_ax , 0 ,
            0 , dt_3/2 * noise_ay , 0 , dt_2 * noise_ay; 

  // 3. Call the Kalman Filter predict() function
  kf_.Predict();
  // 4. Call the Kalman Filter update() function
  kf_.Update(measurement_pack.raw_measurements_);
  //      with the most recent raw measurements_
  
  cout << "x_= " << kf_.x_ << endl;
  cout << "P_= " << kf_.P_ << endl;
}


// STATE PREDICTION
/**
 * We now want to estimate the 2-d position and velocity. So we can use a a state vector.
 * 
 * State Vector:  x = | Px |
 *                    | Py |
 *                    | Vx |
 *                    | Vy |  
 *                  
 * Linear Motion Model: Px' = Px + Vx * Δt + Vpx (Noise)
 *                      Py' = Py + Vy * Δt + Vpy (Noise)
 *                      Vx' = Vx + Vvx (Noise)
 *                      Vy' = Vy + Vvy (Noise)
 *          
 *      
 * State Transition Equation : x' = F * x + noise [From before] is now:
 *  
 *                    | Px' |    |   1   0   Δt  0   |  | Px |   | Vpx |
 *                    | Py' |  = |   0   1   0   Δt  |  | Py | + | Vpy |
 *                    | Vx' |    |   0   0   1   0   |  | Vx |   | Vvx |
 *                    | Vy' |    |   0   0   0   1   |  | Vy |   | Vvy |
 * 
 *  
 **/

//PROCESS COVARIANCE MATRIX
/**
 * Process Covariance Matrix Q - Intuition
 * 
 * In the equation:
 * P' = F * P * F^T + Q <---- The compensate for acceleration
 * 
 * Because our state vector only tracks position and velocity, 
 * we are modeling acceleration as a random noise.
 * 
 * Linear Motion Model: Px' = Px + Vx * Δt + (Ax * Δt^2 /2) (Noise)
 *                      Py' = Py + Vy * Δt + (Ay * Δt^2 /2) (Noise)
 *                      Vx' = Vx + (Ax * Δt) (Noise)
 *                      Vy' = Vy + (Ay * Δt) (Noise)
 * 
 * 
 * Random Accelerator Vector can be expressed as:
 * 
 *      
 *      v = | Vpx | = | Δt^2 /2     0    |  | Ax | =  G * a
 *          | Vpy |   |    0     Δt^2 /2 |  | Ay |
 *          | Vvx |   |    Δt       0    |
 *          | Vvy |   |    0       Δt    |
 *          
 * Based on our noise vector we can define now the new covariance matrix Q. 
 * The covariance matrix is defined as the expectation value of the noise 
 * vector times the noise vector V^T.
 *  
 *     Q = E[ v * v^T] = E[ G * a * a^T * G^T]
 * 
 *     Q = G * Qv * G^T , where Qv = | σ^2[ax] σ[axy]  | and σ[axy] are assumed 0
 *                                   | σ[axy]  σ^2[ay] |
 *      
 *      Thus  Q = | Δt^4/4 * σ^2[ax]    0    Δt^3/2 * σ^2[ax]   0          |
 *                |     0    Δt^4/4 * σ^2[ay]     0       Δt^3/2 * σ^2[ay] |
 *                | Δt^3/2 * σ^2[ax]    0    Δt^2 * σ^2[ax]      0         |
 *                |     0    Δt^3/2 * σ^2[ay]     0       Δt^2 * σ^2[ay]   |
 **/

// LASER MEASUREMENTS - SET-UP LASER MATRICES
/**
 * @ z - the measurement vector. For lidar, z vvtor contains position (x,y)
 * @ H - is the matrix that projects your beliefs about your object's current state
 *        into the measurement space of the senser.
 *        [Fancy way of saying we discard velocity from state variable
 *         so: [px,py,vx,vy] -> [px,py]]
 * 
 *          thus: H = |1, 0, 0, 0|
 *                    |0, 1, 0, 0|
 * 
 *  What does prime mean in px'?
 *          - You have already done the prediction step but habe not done measurement.
 *          - prediction 
 *
 *  Covariance Matrix R for lasers
 *      - 2x2 matrix
 *      
 *  R is the measurement noise covariance matrix:
 *  R = | σ^2[px]      0   |
 *      |    0     σ^2[py] |  	
 * 
 */

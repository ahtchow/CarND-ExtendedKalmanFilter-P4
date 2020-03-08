/**
 * JACOBBIAN MATRIX
 * 
 * To linearize our function h(x) , starting with derivation.
 * If we generalize our h(x) to its derivative h'(x) its called the Jacobian
 * 
 *  f(x) = f(μ) + ∂f(μ)/∂x * (x−μ) 
 * 
 *  A matrix containing all the partial derivatives corresponding to the sensor state vector
 * 
 *         z  = | ρ |           x = | px |
 *              | φ |   and         | py | 
 *              | ρ.|               | vx |
 *                                  | vy |
 * 
 * 
 *      Hj = | px/(px^2 +py^2)^1/2                    py/(py^2 +py^2)^1/2                0                         0         |         
 *           | -py/(px^2 +py^2)                         -px/(py^2 +py^2)                 0                         0         | 
 *           | py(vxpy-vypx/(px^2 +py^2)^3/2    -px(vxpy-vypx/(px^2 +py^2)^3/2  px/(px^2 +py^2)^1/2      py/(py^2 +py^2)^1/2 |
 *          
 *            x or y can not = 0
 *  
 *        
 * 
 **/

#include <iostream>
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {
  /**
   * Compute the Jacobian Matrix
   */

  // predicted state example
  // px = 1, py = 2, vx = 0.2, vy = 0.4
  VectorXd x_predicted(4);
  x_predicted << 1, 2, 0.2, 0.4;

  MatrixXd Hj = CalculateJacobian(x_predicted);

  cout << "Hj:" << endl << Hj << endl;

  return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  if(px <= 0.001 || py <= 0.001){
      cout << "Can not divide by zero" << endl;
  }
  
  // compute the Jacobian matrix
  else{
      float dp_by_dpx = px / sqrt(px*px +py*py);
      float dp_by_dpy = py / sqrt(px*px +py*py);
      float da_by_dpx = -py / (px*px +py*py);
      float da_by_dpy = px / (px*px +py*py);
      float dp_dot_by_px = py*(vx*py - vy*px) / ((px*px + py*py)* sqrt(px*px + py*py));
      float dp_dot_by_py = -px*(vx*py - vy*px) / ((px*px + py*py)* sqrt(px*px + py*py));
      Hj << dp_by_dpx, dp_by_dpy , 0 , 0,
            da_by_dpx , da_by_dpy , 0 , 0,
            dp_dot_by_px, dp_dot_by_px, dp_by_dpx, dp_by_dpy;
      
  }

  return Hj;
}

/**
 * @brief EXTENDED KALMAN FILTER VS KALMAN FILTER
 *  
 * EXTENDED KALMAN FILTER:
 *      
 *      x' = f(x, u) , u = 0
 *      y = z - h(x')
 * 
 * Kalman Filter
 *  Prediction:
 *      x' = Fx + u
 *      P' = FPF^T + Q
 *  Measurement Update:
 *      y = x - Hx'
 *      S = HP'H^T + R
 *      K = P'H^TS^-1
 *      x = x' + K*y
 *      P = (I - K*H)P'
 * 
 *  Very similar, but main difference:
 *      - F matrix replaced by Fj when calculating P' (linearize)
 *      - H matrix in kalman filter replaced by Hj when calculating S, K , P
 *      - to calculate x',  the prediction update function f is used instead of F matrix
 *      - to calculate y, for h function is used instead of H matrix.
 *  Note: y = z- H * x' does not equal z - Hj * x
 */
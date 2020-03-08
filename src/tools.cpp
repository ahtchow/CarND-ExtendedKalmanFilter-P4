#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

const float p_tolerance = 0.001;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
   /**
   * @brief Calculate the RMSE.
   */

   VectorXd rmse(4);
   rmse << 0,0,0,0;

   bool same_size =  estimations.size() == ground_truth.size();
   if(estimations.size() == 0 || !same_size){
    cout << "Error estimation could not calculated." << endl;
    return rmse;
   }

   for (int i=0; i < estimations.size(); ++i) {
      VectorXd x = estimations[i] - ground_truth[i];
      x = x.array() * x.array();
      rmse += x;
   }

   /**  
    *  Root Mean Squared Error:
    *  RMSE = ( 1/n * summation(x_est - x_true)^2 )^1/2
    */
   
   rmse = (rmse / estimations.size());
   rmse = rmse.array().sqrt();
   return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
   /**
   * @brief Calculate a Jacobian.
   */

   MatrixXd Hj(3,4);
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   if(fabs(px) < p_tolerance)
      px = p_tolerance;
   if(fabs(py) < p_tolerance)
      py = p_tolerance;

   // Compute the Jacobian matrix
   float p_divisor = px*px + py*py;
   if(p_divisor < p_tolerance * p_tolerance){
      p_divisor = p_tolerance * p_tolerance;
   }
   
   float dp_by_dpx = px / sqrt(p_divisor);
   float dp_by_dpy = py / sqrt(p_divisor);
   float da_by_dpx = -py / (p_divisor);
   float da_by_dpy = px / (p_divisor);
   float dp_dot_by_px = py*(vx*py - vy*px) / ((p_divisor)* sqrt(p_divisor));
   float dp_dot_by_py = -px*(vx*py - vy*px) / ((p_divisor)* sqrt(p_divisor));
   
   /**      
   *     Hj = | px/(px^2 +py^2)^1/2                    py/(py^2 +py^2)^1/2                0                         0         |         
   *          | -py/(px^2 +py^2)                         -px/(py^2 +py^2)                 0                         0         | 
   *          | py(vxpy-vypx/(px^2 +py^2)^3/2    -px(vxpy-vypx/(px^2 +py^2)^3/2  px/(px^2 +py^2)^1/2      py/(py^2 +py^2)^1/2 |
   **/

   Hj << dp_by_dpx,     dp_by_dpy,    0,         0,
         da_by_dpx,     da_by_dpy,    0,         0,
         dp_dot_by_px,  dp_dot_by_py, dp_by_dpx, dp_by_dpy;


  return Hj;
}

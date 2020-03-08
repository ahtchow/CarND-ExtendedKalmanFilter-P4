// Kalman Filter Equations in C++
/**                          
 *  x - pedestrian state x = | p | matrix
 *                           | v |
 * 
 *  PREDICTION Step:
 * 
 *      1. x' = F*x + noise (State Transion Function)
 * 
 *         noise -> Q (often from accerlation)
 *             
 *              where F = | 1  Δt |
 *                        | 0   1 |
 *      
 *      2. p' = p + v * Δt, (Linear Motion Model)
 * 
 *      we can express the prediction as: | p' | = | 1 Δt | * | p |
 *                                        | v' | = | 0  1 | * | v |
 *  
 *       Multiplication results in: p' = p + Δt * v
 *                                  v' = v
 *      
 * 
 *  UPDATE Step:
 *      
 *      z = H * x + w
 *              z = measurement space of sensor
 *              H = measurement function matrix 
 *              w - sensor noise
 * 
 *      So for lidar, the measurement function looks like this:  z = p'
 * 
 *      It also can be represented in a matrix form:
 * 
 *      z = | 1 0 | | p' |
 *                  | v' |
 * 
 *      As we already know, the general algorithm is composed of a 
 *      prediction step where I predict the new state and covariance, P.
 * 
 *      And we also have a measurement update (or also called many times 
 *      a correction step) where we use the latest measurements to update 
 *      our estimate and our uncertainty.
 *              
 */

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


#include <iostream>
#include "Eigen/Dense"
#include <vector>

using namespace std;
using namespace Eigen;

//Kalman Filter variables:

VectorXd x;	// object state
MatrixXd P;	// object covariance matrix
VectorXd u;	// external motion
MatrixXd F; // state transition matrix
MatrixXd H;	// measurement matrix
MatrixXd R;	// measurement covariance matrix
MatrixXd I; // Identity matrix
MatrixXd Q;	// process covariance matrix

vector <VectorXd> measurements;

void filter(VectorXd &x, MatrixXd &P) {

  for (unsigned int n = 0; n < measurements.size(); ++n) {

    VectorXd z = measurements[n];
    // KF Measurement update step
    VectorXd y = z - (H * x); //Error
    MatrixXd H_t = H.transpose(); 
    MatrixXd S = H * P * H_t + R;
    MatrixXd S_i = S.inverse();
    MatrixXd K = P  * H_t * S_i;
    		 
    // new state from State Transition Function
	  x = x + (K*y);
    P = (I - K*H) * P;

    // KF Prediction step
    x = F * x + u;
    MatrixXd F_t = F.transpose();
    P = F * P * F_t + Q;
		
    cout << "x=" << endl <<  x << endl;
    cout << "P=" << endl <<  P << endl;
  }
}

int main(){
    //HOW TO USE EIGEN LIBRARY
    // VectorXd my_vector(2); // Initalize vertical matrix size 2
    // my_vector << 10, 20;
    // cout << my_vector << endl;
    
    // MatrixXd my_matrix(2,2); // (Row, Column)
    // my_matrix << 1, 2,
    //              3, 4;

    // my_matrix(1,0) = 3;
    // my_matrix(1,1) = 4;
    // cout << my_matrix << endl;

    // VectorXd another_matrix;
    // // (2,2) x (2,1) = (2,1)
    // another_matrix = my_matrix*my_vector;
    // cout << another_matrix << endl;
    
    // another_matrix = another_matrix.transpose();
    // cout << another_matrix << endl;

    // design the KF with 1D motion

    x = VectorXd(2); // | 0 |
    x << 0, 0;       // | 0 |

    P = MatrixXd(2, 2);
    P << 1000, 0,
         0, 1000;

    u = VectorXd(2);
    u << 0,0;

    F = MatrixXd(2, 2);
    F << 1, 1, 0, 1;

    H = MatrixXd(1, 2);
    H << 1, 0;

    R = MatrixXd(1, 1);
    R << 1;

    I = MatrixXd::Identity(2, 2);

    Q = MatrixXd(2, 2);
    Q << 0, 0, 0, 0;

    // Create a list of measurements
    VectorXd single_measurement(1);
    single_measurement << 1;
    measurements.push_back(single_measurement);
    single_measurement << 2;
    measurements.push_back(single_measurement);
    single_measurement << 3;
    measurements.push_back(single_measurement);
    single_measurement << 3;
    measurements.push_back(single_measurement);
    single_measurement << 3;
    measurements.push_back(single_measurement);
    single_measurement << 3;
    measurements.push_back(single_measurement);
    single_measurement << 3;
    measurements.push_back(single_measurement);

    filter(x, P);
    return 0;
}



    


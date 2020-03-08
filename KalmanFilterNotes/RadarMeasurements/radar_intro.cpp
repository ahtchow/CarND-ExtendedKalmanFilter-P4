/** INTEGRATING RADAR MEASUREMENTS FOR DATA FUSION
 *  
 *  Lidar sensors can help accurately detect the position of objects,
 *  but we do not have a method of calculating the velocity.
 * 
 *  Thats where radar data comes into the play.
 *  
 *  LIDAR -> Preprocessing -> Position (No Speed)
 *  RADAR -> Preprocessing -> Radial Velocity 
 * 
 *  
 *  Radar sees in polar coordinate
 * 
 *      1. RANGE: ρ (rho) the radial (shortest) distance to the obbject
 *      2. BEARING: φ (phi) is the angle from cars straight line view
 *      3. RADIAL VELOCITY: ρ. (rho dot) is the change in ρ
 *          φ = atan(py/px)  -> y axis is px and x axis is px
 *          ρ = (px'^2 + py'^2)^1/2 
 *          ρ. = (px'vx' + py'vy')/ (px'^2 + py'^2)^1/2 
 *          
 *    H versus h(x): 
 *      The HH matrix from the lidar lesson and h(x)h(x) equations from the radar 
 *      lesson are actually accomplishing the same thing; 
 *      they are both needed to solve y = z - H * x' in the update step.
 * 
 *      But for radar, there is no HH matrix that will map the state vector xx into polar coordinates; 
 *      instead, you need to calculate the mapping manually to convert from cartesian coordinates to 
 *      polar coordinates.
 *      
 *  MEASUREMENT FUNCTION
 * 
 *      h(x') = | ρ | = |            (px'^2 + py'^2)^1/2          |
 *              | φ |   |                 atan(py'/px')           |
 *              | ρ.|   | (px'vx' + py'vy')/ (px'^2 + py'^2)^1/2  |           
 *      
 *      Hence for radar y = z - Hx' becomes y = z - h(x')
 * 
 *   
 *  What bout not linear motion? We use something called extended kalman filter.
 *          
 *      The first order taylor expansion: to linearize non-linear functions
 *          
 *          f(x) = f(μ) + ∂f(μ)/∂x * (x−μ)
 * 
 *      Simply replace f(x)f(x) with a given equation, find the partial derivative, 
 *      and plug in the value \muμ to find the Taylor expansion at that value of μ.
 *      
 *          For e.g -> h(x) = arctan(x)
 *                     ∂h = 1/ (1 + X^2)
 *            
 *        Calculate the estimate of f(x) using taylor series where μ = 0 and σ = 3.
 *          
 *          f(x) = arctan(0) + 1/ (1 + 0)(x)
 *               = x
 * 
 *          So the function f(x) = arctan(x)  is aprx. f(x) = x
 * 
 *   
 * // HOW DO WE PERFORM MILTI-DIMENSION TAYLOR SERIES
 *  
 *  T(x) = f(a) + (x-a)^T  * Df(a)  + 1/2! (x-a)^T * D^2f(a) (x-a)+ ...
 *  
 *  where Df(a) is called the Jacobian matrix and D^f(a) is the Hessian matrix.
 *  They represent the first order and secord order derivations of multi-dim equation.     
 *          
 *      
 * 
 * 
 * 
 *      
 * 
 */
#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "FusionEKF.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;

using json = nlohmann::json; /** < For convenience */

/**
 * @brief Check if SocketIO event has JSON data.
 * If yes, JSON  string format will be returned
 * else "" will be returned
 */
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;
  FusionEKF fusionEKF; /** < Create a Kalman Filter instance */
  Tools tools; /** < Create a tool instance to calculate RMSE */
  vector<VectorXd> estimations; /** < Estimated state */
  vector<VectorXd> ground_truth; /** < True state */

  h.onMessage([&fusionEKF,&tools,&estimations,&ground_truth]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
               uWS::OpCode opCode) {
    /**
     * @brief "42" at the start of the message means there's a websocket message event.
     * The 4 signifies a websocket message.
     * The 2 signifies a websocket event.
     */
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data)); 

      if (s != "") {
        auto j = json::parse(s); 
        string event = j[0].get<string>(); /** < Read for "telemetry" */
        
        if (event == "telemetry") {
          /**
           * @brief Read Sensor Measurements: 
           * est_px est_py est_vx est_vy meas_px meas_py gt_px gt_py gt_vx gt_vy
           */
          string sensor_measurement = j[1]["sensor_measurement"];

          MeasurementPackage meas_package; /** < Measurement Package contains: Sensor Type, Time Stamp, Measurements. */
          long long timestamp;
          std::istringstream iss(sensor_measurement); 

          string sensor_type; 
          iss >> sensor_type; /** < Stream sensor-type: Lidar or Radar. */
          /**
           * @brief Based on sensor type stream appropriate data.
           * LIDAR: X Postion, Y Position, Time Stamp. 
           * RADAR: Radial Distance (ρ), Bearing (φ), Radial Velocity (ρ.). 
           */
          if (sensor_type.compare("L") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px, py; 
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          } 
          else if (sensor_type.compare("R") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro, theta, ro_dot;
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro, theta, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          }

          float x_gt, y_gt, vx_gt, vy_gt; 
          iss >> x_gt; /** < True x-position. */
          iss >> y_gt; /** < True y-position. */
          iss >> vx_gt; /** < True y-velocity. */
          iss >> vy_gt; /** < True y-velocity. */

          VectorXd gt_values(4); /** < Store Ground Truth */
          gt_values(0) = x_gt;
          gt_values(1) = y_gt; 
          gt_values(2) = vx_gt;
          gt_values(3) = vy_gt;
          ground_truth.push_back(gt_values);
          
          // Call ProcessMeasurement(meas_package) for Kalman filter
          fusionEKF.ProcessMeasurement(meas_package);       

          /** Push the current estimated x,y positon from the Kalman filter's state vector */
          VectorXd estimate(4); /** < Store Estimates */
          double p_x = fusionEKF.ekf_.x_(0);
          double p_y = fusionEKF.ekf_.x_(1);
          double v1  = fusionEKF.ekf_.x_(2);
          double v2 = fusionEKF.ekf_.x_(3);

          estimate(0) = p_x; 
          estimate(1) = p_y;
          estimate(2) = v1;
          estimate(3) = v2;
          estimations.push_back(estimate);

          json RMSEMetrics; /** < Store Root Mean Squared for each variable. */
          VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
          RMSEMetrics["estimate_x"] = p_x;
          RMSEMetrics["estimate_y"] = p_y;
          RMSEMetrics["rmse_x"] =  RMSE(0);
          RMSEMetrics["rmse_y"] =  RMSE(1);
          RMSEMetrics["rmse_vx"] = RMSE(2);
          RMSEMetrics["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + RMSEMetrics.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if

      } else {
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if

  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
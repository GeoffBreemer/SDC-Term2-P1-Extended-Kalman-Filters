#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"

class FusionEKF {
private:
  bool is_initialized_;
  long previous_timestamp_;

  Eigen::MatrixXd R_laser_;   // Measurement covariance matrix - laser measurement noise
  Eigen::MatrixXd R_radar_;   // Measurement covariance matrix - radar measurement noise
  Eigen::MatrixXd H_laser_;   // Measurement function H matrix

  // Process noise
  float noise_ax;
  float noise_ay;

public:
  FusionEKF();
  virtual ~FusionEKF();

  // Run the Predict -> Measurement Update process
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  KalmanFilter ekf_;
};

#endif /* FusionEKF_H_ */

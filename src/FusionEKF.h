#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"

class FusionEKF {
public:
  FusionEKF();

  virtual ~FusionEKF();

  // Run the Predict -> Measurement Update process
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  KalmanFilter ekf_;

private:
  bool is_initialized_;
  long previous_timestamp_;

  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;

  float noise_ax;
  float noise_ay;
};

#endif /* FusionEKF_H_ */

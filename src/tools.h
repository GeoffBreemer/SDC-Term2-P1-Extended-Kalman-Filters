#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"

#define APPROX_ZERO 0.0001

namespace Tools {
  //  Calculate RMSE
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  // Calculate the Jacobian
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  // Convert from polar to cartesian coordinates
  void polar_to_cartesian(const MeasurementPackage& measurement_pack, double& px, double& py);

  // Convert from cartesian to polar coordinates
  Eigen::VectorXd cartesian_to_polar(Eigen::VectorXd x);
};

#endif /* TOOLS_H_ */

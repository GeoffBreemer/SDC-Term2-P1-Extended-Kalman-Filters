#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

namespace Tools {
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                         const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // Check inputs
    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0) {
      cout << "Invalid estimation or ground_truth data" << endl;
      return rmse;
    }

    // Calculate RMSE
    for (int i = 0; i < estimations.size(); ++i) {
      VectorXd res = estimations[i] - ground_truth[i];
      res = res.array() * res.array();
      rmse += res;
    }

    rmse /= estimations.size();
    rmse = rmse.array().sqrt();

    return rmse;
  }

  MatrixXd CalculateJacobian(const VectorXd &x_state) {
    MatrixXd Hj(3, 4);

    // Unpack the state vector
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // Calculate frequently used calculations
    float px2 = px * px;
    float py2 = py * py;
    float ss = px2 + py2;

    // Check division by zero
    if (fabs(ss) < APPROX_ZERO) {
      cout << "CalculateJacobian () - Error - Division by Zero" << endl;
      ss = APPROX_ZERO;
    }

    float srss = sqrt(ss);

    // Create the Jacobian
    Hj(0, 0) = px / srss;
    Hj(0, 1) = py / srss;
    Hj(0, 2) = 0;
    Hj(0, 3) = 0;

    Hj(1, 0) = -py / ss;
    Hj(1, 1) = px / ss;
    Hj(1, 2) = 0;
    Hj(1, 3) = 0;

    Hj(2, 0) = (py * (vx * py - px * vy)) / (ss * srss);
    Hj(2, 1) = (px * (vy * px - py * vx)) / (ss * srss);
    Hj(2, 2) = px / srss;
    Hj(2, 3) = py / srss;

    return Hj;
  }

  void polar_to_cartesian(const MeasurementPackage &measurement_pack, double &px, double &py) {
    double phi = measurement_pack.raw_measurements_[1];

    px = measurement_pack.raw_measurements_[0] * cos(phi);
    py = measurement_pack.raw_measurements_[0] * sin(phi);
  }

  Eigen::VectorXd cartesian_to_polar(const Eigen::VectorXd x) {
    VectorXd z_pred(3);

    // Unpack the state vector
    double px = x(0);
    double py = x(1);
    double vx = x(2);
    double vy = x(3);

    if (fabs(px) < APPROX_ZERO) {
      px = APPROX_ZERO;
    }

    // Convert from cartesian to polar
    float px2 = px * px;
    float py2 = py * py;
    float rho = sqrt(px2 + py2);

    // Avoid division by zero
    if (fabs(rho) < APPROX_ZERO) {
      rho = APPROX_ZERO;
    }

    z_pred[0] = rho;
    z_pred[1] = atan(py / px);
    z_pred[2] = (px * vx + py * vy) / rho;

    return z_pred;
  }
}
#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF() {
  // Initialise variables
  is_initialized_ = false;
  previous_timestamp_ = 0;
  noise_ax = 9;
  noise_ay = 9;

  // Lidar specific constants
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Radar specific constants
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  // Perform one-off initialization using the first measurement
  if (!is_initialized_) {
    double px;
    double py;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar measurement from polar to cartesian coordinates
      Tools::polar_to_cartesian(measurement_pack, px, py);

      // If initial position values are zero use initial guesses
      if(fabs(px) < APPROX_ZERO) {
        px = 1;
        ekf_.P_(0,0) = px;
      }

      if(fabs(py) < APPROX_ZERO) {
        py = 1;
        ekf_.P_(1,1) = py;
      }
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // No conversion required for lidar measurements
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];
    }

    // Set initial state x to the first position measurement and zero velocity
    VectorXd tmpX = VectorXd(4);
    tmpX << px, py, 0, 0;
    ekf_.setx_(tmpX);

    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

   // 1. Prediction step
  // Compute the time elapsed in seconds between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update F and Q
  ekf_.setF_(dt);
  ekf_.setQ_(dt, noise_ax, noise_ay);

  // Predict state vector x and state covariance matrix P
  ekf_.Predict();

  // 2. Measurement update/correction step
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.UpdateEKF(measurement_pack.raw_measurements_, R_radar_, Tools::CalculateJacobian(ekf_.getx_()));

  } else {
    ekf_.Update(measurement_pack.raw_measurements_, R_laser_, H_laser_);
  }
}

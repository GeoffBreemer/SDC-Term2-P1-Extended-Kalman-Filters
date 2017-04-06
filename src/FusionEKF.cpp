#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  VectorXd x_ = VectorXd(4);
  MatrixXd F_ = MatrixXd(4, 4);
  F_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
  MatrixXd Q_ = MatrixXd(4, 4);

  MatrixXd P_ = MatrixXd(4, 4); // Initial state covariance matrix P
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  noise_ax = 9;                 // Process noise
  noise_ay = 9;

  // LASER specific
  R_laser_ = MatrixXd(2, 2);    // Measurement covariance matrix - laser measurement noise
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  H_laser_ = MatrixXd(2, 4);    // Measurement function H matrix
  H_laser_ << 1, 0, 0, 0,
          0, 1, 0, 0;

  // RADAR specific
  R_radar_ = MatrixXd(3, 3);    // Measurement covariance matrix - radar measurement noise
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  Hj_ = MatrixXd(3, 4);         // Jacobian

  // Initialise the Kalman Filter
  ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
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

      // If initial position values are zero use initial guesses and large covariance.
      if(fabs(px) < APPROX_ZERO) {
        px = 1;
        ekf_.P_(0,0) = 1000;
      }

      if(fabs(py) < APPROX_ZERO) {
        py = 1;
        ekf_.P_(1,1) = 1000;
      }
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // No conversion required for lidar measurements
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];
    }

    // Set initial state x to the first position measurement and zero velocity
    ekf_.x_ << px, py, 0, 0;

    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

   // 1. Prediction step
  // Compute the time elapsed in seconds between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update state transition matrix F
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Update process noise covariance matrix Q
  float dt2 = dt * dt;
  float dt3 = dt * dt2;
  float dt4 = dt * dt3;
  ekf_.Q_ << noise_ax * dt4/4, 0, noise_ax*dt3/2, 0,
             0, noise_ay*dt4/4, 0, noise_ay*dt3/2,
             noise_ax * dt3/2, 0, noise_ax*dt2, 0,
             0, noise_ay*dt3/2, 0, noise_ay*dt2;

  // Predict state vector x and state covariance matrix P
  ekf_.Predict();

  // 2. Measurement update/correction step
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.R_ = R_radar_;
    ekf_.H_ = Tools::CalculateJacobian(ekf_.x_);

    // Update the state and covariance matrices
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;

    // Update the state and covariance matrices
    ekf_.Update(measurement_pack.raw_measurements_);
  }
}

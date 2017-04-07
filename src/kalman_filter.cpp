#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

const double LARGE_VAR = 999.0;      // large variance

KalmanFilter::KalmanFilter() {
  // Initialise variables
  x_ = VectorXd(4);

  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

  Q_ = MatrixXd(4, 4);

  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, LARGE_VAR, 0,
        0, 0, 0, LARGE_VAR;
}

KalmanFilter::~KalmanFilter() {}

// Setters
void KalmanFilter::setx_(const Eigen::VectorXd x) {
  x_ = x;
}

void KalmanFilter::setF_(float dt) {
  F_(0, 2) = dt;
  F_(1, 3) = dt;
}

void KalmanFilter::setQ_(float dt, float noise_ax, float noise_ay) {
  float dt2 = dt * dt;
  float dt3 = dt * dt2;
  float dt4 = dt * dt3;

  Q_ << noise_ax * dt4/4, 0, noise_ax*dt3/2, 0,
        0, noise_ay*dt4/4, 0, noise_ay*dt3/2,
        noise_ax * dt3/2, 0, noise_ax*dt2, 0,
        0, noise_ay*dt3/2, 0, noise_ay*dt2;
}

// Getters
Eigen::VectorXd KalmanFilter::getx_() {
  return x_;
}

void KalmanFilter::Predict() {
  // Predict the state
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z, const Eigen::MatrixXd &R, const Eigen::MatrixXd &H) {
  // Update the state using Kalman Filter equations (for lidar measurements)
  R_ = R;
  H_ = H;   // H should be the measurement matrix

  VectorXd z_pred = H * x_;
  Estimate(z, z_pred);
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const Eigen::MatrixXd &R, const Eigen::MatrixXd &Hj) {
  // Update the state using Kalman Filter equations (for radar measurements)
  R_ = R;
  H_ = Hj;  // H should be the Jacobian

  // Convert current state from cartesian to polar
  VectorXd z_pred = Tools::cartesian_to_polar(x_);

  Estimate(z, z_pred);
}

void KalmanFilter::Estimate(const Eigen::VectorXd &z, const Eigen::VectorXd &z_pred) {
  // Perform various matrix calculations
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();     // H_ contains the Jacobian (for radar) or H matrix (for lidar)
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // New estimates for x and P
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
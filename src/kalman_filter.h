#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
private:
  Eigen::MatrixXd H_;   // measurement matrix
  Eigen::MatrixXd R_;   // measurement covariance matrix
  Eigen::VectorXd x_;   // state vector
  Eigen::MatrixXd F_;   // state transition matrix
  Eigen::MatrixXd Q_;   // process noise covariance matrix
  Eigen::MatrixXd P_;   // state covariance matrix

  // Calculates the actual estimates of x and P
  void Estimate(const Eigen::VectorXd &z, const Eigen::VectorXd &z_pred);

public:
  KalmanFilter();
  virtual ~KalmanFilter();

  // Setters
  void setx_(const Eigen::VectorXd x);
  void setF_(float dt);
  void setQ_(float dt, float noise_ax, float noise_ay);

  // Getters
  Eigen::VectorXd getx_();

  // Predicts the state and the state covariance
  void Predict();

  // Updates the state by using standard Kalman Filter equations (for Lidar measurements)
  void Update(const Eigen::VectorXd &z, const Eigen::MatrixXd &R, const Eigen::MatrixXd &H);

  // Updates the state by using Extended Kalman Filter equations (for Radar measurements)
  void UpdateEKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &R, const Eigen::MatrixXd &Hj);
};

#endif /* KALMAN_FILTER_H_ */

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:

  Eigen::VectorXd x_;   // state vector
  Eigen::MatrixXd P_;   // state covariance matrix
  Eigen::MatrixXd F_;   // state transition matrix
  Eigen::MatrixXd Q_;   // process covariance matrix
  Eigen::MatrixXd H_;   // measurement matrix
  Eigen::MatrixXd R_;   // measurement covariance matrix

  KalmanFilter();
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  // Predicts the state and the state covariance
  void Predict();

  // Updates the state by using standard Kalman Filter equations (for Lidar measurements)
  void Update(const Eigen::VectorXd &z);

   // Updates the state by using Extended Kalman Filter equations (for Radar measurements)
  void UpdateEKF(const Eigen::VectorXd &z);

  void Estimate(const Eigen::VectorXd &z, const Eigen::VectorXd &z_pred);
};

#endif /* KALMAN_FILTER_H_ */

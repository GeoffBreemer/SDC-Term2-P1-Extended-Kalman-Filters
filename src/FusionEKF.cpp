#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  //measurement covariance matrix - laser measurement noise
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar measurement noise
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Eigen::MatrixXd Hj_;
  Hj_ = MatrixXd(3, 4);


  // Process noise
  noise_ax = 9;
  noise_ay = 9;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.Q_ = MatrixXd(4, 4);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    // Set initial state x to the first measurement and zero velocity
    ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    // Set the initial state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update F for dt
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // Update Q for dt
  float dt2 = dt * dt;
  float dt3 = dt * dt2;
  float dt4 = dt * dt3;

  ekf_.Q_ << noise_ax * dt4/4, 0, noise_ax*dt3/2, 0,
             0, noise_ay*dt4/4, 0, noise_ay*dt3/2,
             noise_ax * dt3/2, 0, noise_ax*dt2, 0,
             0, noise_ay*dt3/2, 0, noise_ay*dt2;


  // Predict state vector x and P
  ekf_.Predict();

  cout << "predicting\n";

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates - TODO
    ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, H_laser_, R_radar_, ekf_.Q_);

  } else {
    ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, H_laser_, R_laser_, ekf_.Q_);
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

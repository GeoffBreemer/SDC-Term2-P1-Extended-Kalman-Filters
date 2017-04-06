#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE
  if(estimations.size() != ground_truth.size()
     || estimations.size() == 0){
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    VectorXd res = estimations[i] - ground_truth[i];
    res = res.array() * res.array();
    rmse += res;
  }

  //calculate the mean
  rmse /= estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  MatrixXd Hj(3,4);

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //compute the Jacobian matrix
  float px2 = px * px;
  float py2 = py * py;
  float ss = px2 + py2;
  float srss = sqrt(ss);

  //check division by zero
  if(fabs(ss) < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  //TODO: YOUR CODE HERE
  Hj(0,0) = px / srss;
  Hj(0,1) = py / srss;
  Hj(0,2) = 0;
  Hj(0,3) = 0;

  Hj(1,0) = -py / ss;
  Hj(1,1) = px / ss;
  Hj(1,2) = 0;
  Hj(1,3) = 0;

  Hj(2,0) = (py * (vx * py - px * vy)) / (ss * srss);
  Hj(2,1) = (px * (vy * px - py * vx)) / (ss * srss);
  Hj(2,2) = px / srss;
  Hj(2,3) = py / srss;

  return Hj;
}

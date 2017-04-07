#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "ground_truth_package.h"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // Make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {

  // Check files and parameters
  check_arguments(argc, argv);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  vector<MeasurementPackage> measurement_pack_list;     // measurements
  vector<GroundTruthPackage> gt_pack_list;              // ground truths
  string line;

  // Prepare measurement packages for each line in the file
  while (getline(in_file_, line)) {
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    string sensor_type;

    istringstream iss(line);
    long long timestamp;

    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) {
      // Read laser measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);

      float x, y;
      iss >> x;
      iss >> y;

      meas_package.raw_measurements_ << x, y;
    } else if (sensor_type.compare("R") == 0) {
      // Read radar measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);

      float ro, phi, ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;

      meas_package.raw_measurements_ << ro, phi, ro_dot;
    }

    iss >> timestamp;
    meas_package.timestamp_ = timestamp;
    measurement_pack_list.push_back(meas_package);

    // Read ground truth data to compare later
    float x_gt, y_gt, vx_gt, vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;

    gt_package.gt_values_ = VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
    gt_pack_list.push_back(gt_package);
  }

  // Create a Fusion EKF instance
  FusionEKF fusionEKF;

  // Used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // Apply the Kalman Filter to each measurement package
  size_t N = measurement_pack_list.size();
  for (size_t k = 0; k < N; ++k) {
    // Process the current measurement
    fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

    // Output the estimation
    out_file_ << fusionEKF.ekf_.getx_()(0) << "\t";
    out_file_ << fusionEKF.ekf_.getx_()(1) << "\t";
    out_file_ << fusionEKF.ekf_.getx_()(2) << "\t";
    out_file_ << fusionEKF.ekf_.getx_()(3) << "\t";

    // Output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      // First convert measurement from polar to cartesian
      double px, py;
      Tools::polar_to_cartesian(measurement_pack_list[k], px, py);
      out_file_ << px << "\t";
      out_file_ << py << "\t";
    }

    // Output the ground truth packages
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\n";

    estimations.push_back(fusionEKF.ekf_.getx_());
    ground_truth.push_back(gt_pack_list[k].gt_values_);
  }

  // Compute the accuracy (RMSE)
  cout << "Accuracy - RMSE:" << endl << Tools::CalculateRMSE(estimations, ground_truth) << endl;

  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  return 0;
}

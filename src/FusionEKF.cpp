#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#define THRESHOLD 0.0001

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
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ <<  1, 1, 0, 0,
          1, 1, 0, 0,
          1, 1, 1, 1;

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ <<  1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;

  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ <<  1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() = default;

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
    ekf_.x_ << 1, 1, 5, 0.5;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double rho_dot = measurement_pack.raw_measurements_[2];
      double px = rho*cos(phi);
      double py = rho*sin(phi);
      double vx = rho_dot*cos(phi);
      double vy = rho_dot*sin(phi);
      ekf_.x_ << px, py, vx, vy;
      cout << "[RADAR] Initializing x: " << endl << ekf_.x_ << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], ekf_.x_(2), ekf_.x_(3);
      cout << "[LASER] Initializing x: " << endl << ekf_.x_ << endl;
    }
    previous_timestamp_ = measurement_pack.timestamp_;
    // If the initial values are less than THRESHOLD
    if (fabs(ekf_.x_[0]) < THRESHOLD && fabs(ekf_.x_[1]) < THRESHOLD) {
      ekf_.x_[0] = THRESHOLD;
      ekf_.x_[1] = THRESHOLD;
    }

    // The initial timestamp for dt calculation
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
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0f;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  //1. Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //2. Set the process covariance matrix Q
  double dt2 = dt * dt;
  double dt3 = dt * dt2;
  double dt4 = dt * dt3;
  double noise_ax = 9;
  double noise_ay = 9;
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
              0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
              dt3/2*noise_ax, 0, dt2*noise_ax, 0,
              0, dt3/2*noise_ay, 0, dt2*noise_ay;
  //3. Call the Kalman Filter predict() function
  ekf_.Predict();

  // print the output
  //cout << "\n\n+ PREDICT:" << endl;
  //cout << ">> x_ = \n" << ekf_.x_ << endl;
  //cout << ">> P_ = \n" << ekf_.P_ << endl << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  //cout << "+ UPDATE:" << endl;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    //cout << "[R]" << endl;
//    Hj_ = tools.CalculateJacobian(ekf_.x_);
//    ekf_.H_ = Hj_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    //cout << "[L]" << endl;
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << ">> x_ = \n" << ekf_.x_ << endl;
  //cout << ">> P_ = \n" << ekf_.P_ << endl << endl;
}

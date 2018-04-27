#include "kalman_filter.h"
#include <cmath>
#define THRESHOLD 0.0001

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter()
{
  this->Init(
      VectorXd::Constant(4, 0.0),    // x_in
      MatrixXd::Constant(4, 4, 0.0), // P_in
      MatrixXd::Constant(4, 4, 0.0), // F_in
      MatrixXd::Constant(3, 4, 0.0), // H_in
      MatrixXd::Constant(3, 3, 0.0), // R_in
      MatrixXd::Constant(4, 4, 0.0)  // Q_in
  );
}

KalmanFilter::~KalmanFilter() = default;

void KalmanFilter::Init(const VectorXd &x_in, const MatrixXd &P_in, const MatrixXd &F_in,
                        const MatrixXd &H_in, const MatrixXd &R_in, const MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  * predict the state
  * x′ = F∗x + noise (noise = 0)
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  UpdateKF(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  * update the state by using Extended Kalman Filter equations
  */
  double rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  double phi = atan2(x_(1), x_(0));
  double rho_dot;
  rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;

//  if (fabs(rho) < THRESHOLD) {
//      rho_dot = THRESHOLD;
//  } else {
//      rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
//  }
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;

  // Make sure the angle is still between pi and -pi
  while ( y(1) > M_PI || y(1) < -M_PI ) {
    if (y(1) > M_PI) {
      y(1) -= M_PI;
    } else {
      y(1) += M_PI;
    }
  }
  UpdateKF(y);
}

void KalmanFilter::UpdateKF(const VectorXd &y) {

  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  auto x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

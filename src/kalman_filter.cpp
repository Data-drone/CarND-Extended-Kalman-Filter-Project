#include "kalman_filter.h"
#include "math.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in; // updated in function
  P_ = P_in; // updated in function
  F_ = F_in; // fixed from outside
  H_ = H_in; // specified prior to execution of update
  R_ = R_in; // specified prior to execution of update
  Q_ = Q_in; // fixed from outside
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  //VectorXd predict_x;
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  // intermediate processing

  // h(x)
  VectorXd hx = VectorXd(3);
  double rho, phi, rho_dot = 0.0;
  // detect small ps
  if (x_[0]  == 0) {
    x_[0] += 0.000001;
  }

  if (x_[1]  == 0) {
    x_[1] += 0.000001;
  }

  double denom = x_[0]*x_[0] + x_[1]*x_[1];
  if (denom == 0.0) {
    denom += 0.000001;
  }

  rho = sqrt( denom );
  
  phi = atan2(x_[1], x_[0]);

  // normalise angles
  // move later

  rho_dot = (x_[0]*x_[2] + x_[1]*x_[3] )/sqrt( denom );

  hx << rho, phi, rho_dot;

  VectorXd y = z - hx;

  while (y[1] > M_PI || y[1] < -M_PI ) 
  {
    if (y[1] > M_PI) {
      y[1] -= 2*M_PI;
    } else {
      y[1] += 2*M_PI;
    }
  }


  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

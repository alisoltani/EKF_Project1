#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

  VectorXd y = z - H_ * x_;

  UpdateKF(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // change the x and y coordinates
  double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  double theta = atan2( x_(1) ,  x_(0));
  double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;

  VectorXd h = VectorXd(3); // h(x_)
  h << rho, theta, rho_dot;

  VectorXd y = z - h;

  y[1] = NormalizeAngle(y[1]);

  UpdateKF(y);
}

//
void KalmanFilter::UpdateKF(const VectorXd &y){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

double KalmanFilter::NormalizeAngle(double phi)
{
  const double Max = M_PI;
  const double Min = -M_PI;

  return phi < Min
    ? Max + std::fmod(phi - Min, Max - Min)
    : std::fmod(phi - Min, Max - Min) + Min;
}

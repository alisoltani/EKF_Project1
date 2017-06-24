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

  laser_var_ = 0.0225;
  radar_var_ = 0.09;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // measurement covariance matrix - laser
  R_laser_ << laser_var_, 0,
        0, laser_var_;

  // measurement covariance matrix - radar
  R_radar_ << radar_var_, 0, 0,
        0, radar_var_/100, 0,
        0, 0, radar_var_;

  // measurement matrix for the laser (only uses x and y, can't measure speed vx and vy)
  H_laser_ << 1, 0, 0, 0,
             0, 1, 0, 0;

  noise_ax_ = 9;
  noise_ay_ = 9;

  // initialize the rest of the matrices

  // state covariance matrix
  ekf_.P_ = MatrixXd(4, 4);
  // process covariance matrix
  ekf_.Q_ = MatrixXd(4, 4);
  // transition matrix
  ekf_.F_ = MatrixXd(4, 4);



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
    // first measurement
    cout << "EKF for first measurement: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // polar to cartesian
      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double rho_dot = measurement_pack.raw_measurements_[2];

      ekf_.x_ << rho*cos(phi), rho*sin(phi), 0.0, 0.0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.0, 0.0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // initalize dt to one for the first data because we don't have a timestamp
    int dt = 0.1;
    ekf_.F_ <<   1  ,  0  ,  dt ,  0  ,
                 0  ,  1  ,  0  ,  dt ,
                 0  ,  0  ,  1  ,  0  ,
                 0  ,  0  ,  0  ,  1  ;
    // initial uncertainty, 10 for the position x/y and 1000 for the speed
    ekf_.P_ <<   1  ,  0  ,  0  ,  0  ,
                 0  ,  1  ,  0  ,  0  ,
                 0  ,  0  , 1000,  0  ,
                 0  ,  0  ,  0  , 1000;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // elapsed time, converted to seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // precalculate dt^2, dt^3 and dt^4 for faster execution
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  // adding the elapsed time to the process matrix
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // new process covariance matrix is
  ekf_.Q_ << dt_4/4*noise_ax_,         0       , dt_3/2*noise_ax_,         0       ,
                     0       , dt_4/4*noise_ay_,         0       , dt_3/2*noise_ay_,
             dt_3/2*noise_ax_,         0       ,  dt_2*noise_ax_ ,         0       ,
                     0       , dt_3/2*noise_ay_,         0       ,  dt_2*noise_ay_ ;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar uses EKF
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser uses KF
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

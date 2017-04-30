#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.3; // 1.4 m/s^2

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.515; // 0.52 rad/s^2

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
   * Complete the initialization. See ukf.h for other member properties.
   */

  is_initialized_ = false;
  
  ///* State dimension
  n_x_ = 5;
  
  ///* Augmented state dimension
  n_aug_ = 7;

  // Matrix to hold sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  ///* Sigma point spreading parameter
  lambda_ = 3-n_x_; // controllare

  ///* state covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_ <<   1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
  
  ///* Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);

  weights_(0) = lambda_/(lambda_+n_aug_);

  for (int i=1; i<2*n_aug_+1; i++) {
      weights_(i) = 0.5/(lambda_+n_aug_);
  }
  
  
  ///* the current NIS for radar
  NIS_radar_ = 0;

  ///* the current NIS for laser
  NIS_laser_ = 0;

  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * Complete this function! Make sure you switch between lidar and radar
   * measurements.
  */
  if (!is_initialized_) {

    previous_timestamp_ = meas_package.timestamp_;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

      // init RADAR
      float px = meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(1));
      float py = meas_package.raw_measurements_(0) * sin(meas_package.raw_measurements_(1));
      float v = meas_package.raw_measurements_(2);
      float yaw = meas_package.raw_measurements_(1);
      x_ << px, py, 0, 0, 0;
      is_initialized_ = true;  
      
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

      // init LIDAR
      float px = meas_package.raw_measurements_(0);
      float py = meas_package.raw_measurements_(1);
      x_ << px, py, 0, 0, 0;
      is_initialized_ = true;
    }

    return;
  }

  double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0; // delta t in seconds

  // do the prediction
  Prediction(dt);
   
  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_) {
    UpdateRadar(meas_package);
    
  } else if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_) {
    UpdateLidar(meas_package);
  }

  // update timestamp
  previous_timestamp_ = meas_package.timestamp_;

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
   * Complete this function! Estimate the object's location. Modify the state
   * vector, x_. Predict sigma points, the state, and the state covariance matrix.
   */

  // sigma points
  VectorXd x_aug = VectorXd(7);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;

  // process noise
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;
  MatrixXd sqrt_P_aug= P_aug.llt().matrixL();

  // augmented sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  Xsig_aug.col(0) = x_aug;

  for (int i=0; i<n_aug_; i++) {
      Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_)*sqrt_P_aug.col(i);
      Xsig_aug.col(n_aug_+i+1) = x_aug - sqrt(lambda_+n_aug_)*sqrt_P_aug.col(i);
  }

  // predict sigma points
  for (int i=0; i<2*n_aug_+1; i++) {

    double px = Xsig_aug(0,i);
    double py = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawRate = Xsig_aug(4,i);
    double v_acc_noise = Xsig_aug(5,i);
    double yaw_acc_noise = Xsig_aug(6,i);

    double px_pred;
    double py_pred;
    double v_pred;
    double yaw_pred;
    double yawRate_pred;

    double delta_t_sq = delta_t*delta_t;

    yaw_pred = yaw + yawRate*delta_t;

    if (fabs(yawRate) < 0.001) {
      px_pred = px + (v*cos(yaw)*delta_t);
      py_pred = py + (v*sin(yaw)*delta_t);
    } else {
      px_pred = px + ((v/yawRate) * (sin(yaw_pred)-sin(yaw)));
      py_pred = py + ((v/yawRate) * (-cos(yaw_pred)+cos(yaw)));
    }

    // noise
    px_pred  += 0.5 * delta_t_sq * cos(yaw) * v_acc_noise;
    py_pred  += 0.5 * delta_t_sq * sin(yaw) * v_acc_noise;
    yaw_pred += 0.5 * delta_t_sq * yaw_acc_noise;
    yawRate_pred = yawRate + delta_t * yaw_acc_noise;
    v_pred = v + delta_t * v_acc_noise;

    // assign predictions
    Xsig_pred_(0,i) = px_pred;
    Xsig_pred_(1,i) = py_pred;
    Xsig_pred_(2,i) = v_pred;
    Xsig_pred_(3,i) = yaw_pred;
    Xsig_pred_(4,i) = yawRate_pred;
  }

  // predict state
  x_.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  
  // predict and update state covariance matrix

  P_.fill(0.0);

  for (int i=0; i<2*n_aug_+1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = Normalize(x_diff(3));
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * Use lidar data to update the belief about the object's
   * position. Modify the state vector, x_, and covariance, P_.
   * You'll also need to calculate the lidar NIS.
  */

   // number of lidar measurement elements
  int n_z = 2;

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  for (int i=0; i < 2*n_aug_+1; i++) {
    double px_pred = Xsig_pred_(0,i);
    double py_pred = Xsig_pred_(1,i);
    Zsig(0,i) = px_pred;
    Zsig(1,i) = py_pred;
  }

  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  
  // calculate measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  MatrixXd R = MatrixXd(n_z, n_z);

  R << std_laspx_*std_laspx_, 0,  0, std_laspy_ * std_laspy_;

  S.fill(0.0);

  for (int i=0; i < 2 * n_aug_+1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  S = S + R;
  
  // update x and P
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    x_diff(3) = Normalize(x_diff(3));    
    Tc = Tc + weights_(i)*x_diff*z_diff.transpose();
  }
  
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  //update state and covariance matrix
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
  
  // calculate NIS
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * Use radar data to update the belief about the object's
   * position. Modify the state vector, x_, and covariance, P_.
   * You'll also need to calculate the radar NIS.
  */

   // number of radar measurement elements
  int n_z = 3;

  VectorXd z_pred = VectorXd(n_z);
  
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  for (int i=0; i < 2 * n_aug_+1; i++) {
    double px_pred = Xsig_pred_(0,i);
    double py_pred = Xsig_pred_(1,i);
    double v_pred = Xsig_pred_(2,i);
    double yaw_pred = Xsig_pred_(3,i);
    double yawDot_pred = Xsig_pred_(4,i);
    double p_pred_meas = sqrt(px_pred*px_pred + py_pred*py_pred);

    if (p_pred_meas < 0.00001) {
      p_pred_meas = 0.00001;
    }

    double rho_pred_meas = 0;
    double p_dot_pred_meas = 0;

    rho_pred_meas = atan2(py_pred,px_pred);
    p_dot_pred_meas = ((px_pred*cos(yaw_pred)*v_pred) + (py_pred*sin(yaw_pred)*v_pred)) / p_pred_meas;

    Zsig(0,i) = p_pred_meas;
    Zsig(1,i) = rho_pred_meas;
    Zsig(2,i) = p_dot_pred_meas;
  }

  // calculate mean predicted measurement
  z_pred.fill(0.0);

  for (int i=0; i < 2 * n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  
  // calculate measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  MatrixXd R = MatrixXd(n_z, n_z);

  R << std_radr_*std_radr_, 0,                       0,
       0                  , std_radphi_*std_radphi_, 0,
       0                  , 0                         , std_radrd_*std_radrd_;

  S.fill(0.0);

  for (int i=0; i < 2 * n_aug_+1; i++) {

    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    z_diff(1) = Normalize(z_diff(1));
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  S = S + R;
  
  // update x and P

  MatrixXd Tc = MatrixXd(n_x_, n_z);

  Tc.fill(0.0);

  for (int i=0; i<2 * n_aug_+1; i++) {

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    x_diff(3) = Normalize(x_diff(3));
    z_diff(1) = Normalize(z_diff(1));
    
    Tc = Tc + weights_(i)*x_diff*z_diff.transpose();
  }
  
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  //update state mean and covariance matrix
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
  z_diff(1) = Normalize(z_diff(1));
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
  
  // calculate NIS
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}


// utility method to normalize angle
double UKF::Normalize(const double rad) {  

  const double Max = M_PI;  
  const double Min = -M_PI;  

  return rad < Min ? Max + std::fmod(rad - Min, Max - Min) : std::fmod(rad - Min, Max - Min) + Min; 

}

#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>


/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;      

  // State dimension
  n_x_ = 5; 

  // Augmented state dimension
  n_aug_ = 7;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 4;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // time when the state is true, in us
  time_us_ = 0.0;

  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;
  
  // Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  double weight = 0.5/(lambda_ + n_aug_);
  for (int i = 1; i < 2*n_aug_+1; ++i) weights_(i) = weight;
  
  sqrtTerm_ = sqrt(lambda_ + n_aug_);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_)
    {
        // cout << "Kalman Filter Initialization " << endl;

        // set the state with the initial location and zero velocity
        // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad

        if (meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            x_ << meas_package.raw_measurements_(0),
                meas_package.raw_measurements_(1),
                0.0,
                0.0,
                0.0;
            P_ = MatrixXd::Identity(n_x_, n_x_);
            P_(2, 2) = 1.5;
            P_(3, 3) = 1.5;
            P_(4, 4) = 1.5;
            std::cout << "Initialized by Lidar" << std::endl;
        }
        else
        {
            double rho = meas_package.raw_measurements_(0);
            double phi = meas_package.raw_measurements_(1);
            double rho_dot = meas_package.raw_measurements_(2);

            double px = rho * cos(phi);
            double py = rho * sin(phi);

            x_ << px,
                py,
                0.0,
                0.0,
                0.0;
            P_ = MatrixXd::Identity(n_x_, n_x_);
            std::cout << "Initialized by Radar" << std::endl;
        }
        is_initialized_ = true;
        return;
    }
    
    // compute the time elapsed between the current and previous measurements
    // dt - expressed in seconds
    float dt = (meas_package.timestamp_ - time_us_) / 1000000.0; // microseconds to seconds

    Prediction(dt);

    if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
    {
        UpdateRadar(meas_package);
    }
    if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
    {
        UpdateLidar(meas_package);
    }

    time_us_ = meas_package.timestamp_;
}

void UKF::AugmentedSigmaPoints(MatrixXd *Xsig_out)
{
    // create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);

    // create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

    // create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    // create augmented mean state
    x_aug << x_, 0, 0; // since nu_a, nu_yawAccel mean is zero

    // create augmented covariance matrix
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug.topRightCorner(n_x_, n_aug_ - n_x_) = MatrixXd::Zero(n_x_, n_aug_ - n_x_);
    P_aug.bottomLeftCorner(n_aug_ - n_x_, n_x_) = MatrixXd::Zero(n_aug_ - n_x_, n_x_);
    MatrixXd Q(n_aug_ - n_x_, n_aug_ - n_x_);  Q << std_a_*std_a_, 0, 0, std_yawdd_*std_yawdd_;
    P_aug.bottomRightCorner(n_aug_ - n_x_, n_aug_ - n_x_) = Q;

    // create square root matrix
    MatrixXd A = P_aug.llt().matrixL();

    // create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    for(int i = 0; i < n_aug_; ++i)
    {
        Xsig_aug.col(i+1) = x_aug + sqrtTerm_ * A.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrtTerm_ * A.col(i);
    }

    // write result
    *Xsig_out = Xsig_aug;
}

VectorXd UKF::processModel(VectorXd x, double delta_t)
{
    VectorXd xNext(n_x_);
    double px = x(0), py = x(1);
    double v = x(2), yawAngle = x(3), yawRate = x(4);
    double nu_a = x(5), nu_yawAccel = x(6);

    VectorXd f(n_x_);
    VectorXd noise(n_x_);
    if(fabs(yawRate) > 1E-3) // yawRate is not zero
    {
        f(0) = v/yawRate*(sin(yawAngle+yawRate*delta_t) - sin(yawAngle));
        f(1) = v/yawRate*(-cos(yawAngle+yawRate*delta_t) + cos(yawAngle));
        f(2) = 0.0;
        f(3) = yawRate*delta_t;
        f(4) = 0.0;
    }
    else
    {
        f(0) = v*cos(yawAngle)*delta_t;
        f(1) = v*sin(yawAngle)*delta_t;
        f(2) = 0.0;
        f(3) = 0.0;
        f(4) = 0.0;
    }

    noise(0) = 0.5*delta_t*delta_t*cos(yawAngle)*nu_a;
    noise(1) = 0.5*delta_t*delta_t*sin(yawAngle)*nu_a;
    noise(2) = delta_t*nu_a;
    noise(3) = 0.5*delta_t*delta_t*nu_yawAccel;
    noise(4) = delta_t*nu_yawAccel;
    xNext = x.head(n_x_) + f + noise; 

    return xNext;   
    
}

void UKF::SigmaPointPrediction(MatrixXd &Xsig_aug, double delta_t) {

    // predict sigma points
    // avoid division by zero
    // write predicted sigma points into right column
    for (int i = 0; i < 2*n_aug_+1; ++i)
    {   
        VectorXd x = Xsig_aug.col(i);
        Xsig_pred_.col(i) = processModel(x, delta_t);
    }
}

void UKF::PredictMeanAndCovariance() {

    // create vector for predicted state
    VectorXd x = VectorXd(n_x_);

    // create covariance matrix for prediction
    MatrixXd P = MatrixXd(n_x_, n_x_);

    x.fill(0.0);
    for (int i = 0 ; i < 2*n_aug_+1; ++i) x += weights_(i) * Xsig_pred_.col(i); // predict state mean
    x_ = x;

    P.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    { 
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x; // x_diff(3) = yawAngle_diff
        // angle normalization
        while (x_diff(3) > M_PI)  x_diff(3) -= 2. * M_PI; 
        while (x_diff(3) < -M_PI)  x_diff(3) += 2. * M_PI;

        // predict state covairance
        P += weights_(i) * x_diff * x_diff.transpose();
    }
    P_ = P;
}


void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  MatrixXd Xsig_aug(n_aug_, 2*n_aug_+1);
  AugmentedSigmaPoints(&Xsig_aug); // generate sigma points

  SigmaPointPrediction(Xsig_aug, delta_t); // predict sigma points

  PredictMeanAndCovariance(); // predict mean and covariance of sigma points
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
    /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
    int n_z = 2;
    VectorXd z(n_z);
    z << meas_package.raw_measurements_[0],
        meas_package.raw_measurements_[1];

    // measurement covariance
    MatrixXd R_ = MatrixXd(2, 2);
    R_ << std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_;

    // measurement matrix
    MatrixXd H_ = MatrixXd(2, 5);
    H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;

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

    double NIS_value = (z - z_pred).transpose() * Si * (z - z_pred);
    lidarNIS.push_back(NIS_value);
}

VectorXd UKF::measurementModelRadar(VectorXd x, int n_z)
{
    double px = x(0), py = x(1);
    double v = x(2), yawAngle = x(3), yawRate = x(4);
    
    double rho = sqrt(px*px + py*py);
    double phi = atan2(py,px);
    double rho_dot = (px*cos(yawAngle)*v + py*sin(yawAngle)*v) / rho;

    VectorXd z(n_z);
    z << rho, phi, rho_dot;

    return z;
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) 
{
    // set measurement dimension, radar can measure rho, phi, and rho_dot
    int n_z = 3;

    // create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);

    // transform sigma points into measurement space
    for (int i = 0; i < 2*n_aug_+1; ++i) Zsig.col(i) = measurementModelRadar(Xsig_pred_.col(i), n_z);


    // calculate mean predicted measurement
    z_pred.fill(0.0);
    for (int i = 0; i < 2*n_aug_+1; ++i) z_pred += weights_(i)*Zsig.col(i);

    // calculate innovation covariance matrix S
    S.fill(0.0);
    for (int i = 0; i < 2*n_aug_+1; ++i)
    {
        // angle normalization
        VectorXd diff = Zsig.col(i) - z_pred;
        while (diff(1) > M_PI) diff(1) -= 2.0*M_PI;
        while (diff(1) < -M_PI) diff(1) += 2.0*M_PI;

        S += weights_(i)*diff*(diff.transpose());
    }
    MatrixXd R(n_z, n_z);
    R(0,0) = std_radr_ * std_radr_; R(1,1) = std_radphi_ * std_radphi_; R(2,2) = std_radrd_ * std_radrd_;
    S += R;

    // write result
    *z_out = z_pred;
    *S_out = S;
    *Zsig_out = Zsig;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

    if (meas_package.sensor_type_ != meas_package.RADAR) std::cout << "Error has occured in UpdateRadar" << std::endl;

    // set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;

    // create matrix with sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
   
    // create vector for mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);

    // create example matrix for predicted measurement covariance
    MatrixXd S = MatrixXd(n_z,n_z);

    PredictRadarMeasurement(&z_pred, &S, &Zsig); // predict radar measurement

    // create vector for incoming radar measurement
    VectorXd z = meas_package.raw_measurements_;

    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    // calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2*n_aug_+1; ++i)
    {
        VectorXd diff_x = Xsig_pred_.col(i) - x_;
        while (diff_x(3) > M_PI) diff_x(3) -= 2*M_PI;
        while (diff_x(3) < -M_PI) diff_x(3) += 2*M_PI;

        VectorXd diff_z = Zsig.col(i) - z_pred;
        while (diff_z(1) > M_PI) diff_z(1) -= 2*M_PI;
        while (diff_z(1) < -M_PI) diff_z(1) += 2*M_PI;

        Tc += weights_(i) * diff_x * (diff_z.transpose());
    }
    // calculate Kalman gain K
    MatrixXd kalmanGain = Tc * S.inverse();

    // update state mean and covariance matrix
    VectorXd z_diff = z - z_pred;
    while (z_diff(1) > M_PI) z_diff(1) -= 2*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2*M_PI;
    x_ = x_ + kalmanGain * z_diff;
    P_ = P_ - kalmanGain * S * kalmanGain.transpose();

    double NIS_value = (z - z_pred).transpose() * S.inverse() * (z - z_pred);
    radarNIS.push_back(NIS_value);
}
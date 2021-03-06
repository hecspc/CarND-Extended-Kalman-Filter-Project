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
    
    Hj_ << 1,1,0,0,
           1,1,0,0,
           1,1,1,1;
    
    MatrixXd P_ = MatrixXd(4, 4);
    P_ <<   1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
    
    MatrixXd F_ = MatrixXd(4, 4);
    F_ << 1,0,1,0,
          0,1,0,1,
          0,0,1,0,
          0,0,0,1;
    MatrixXd Q_ = MatrixXd(4, 4);
    
    VectorXd x_ = VectorXd(4);
    x_ << 1, 1, 1, 1;
    
    ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
    
    noise_ax = 9;
    noise_ay = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    
    cout << "------------------------------------------" << endl;
    
//    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
//        return;
//    }
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
      
    // Avoiding dividing by zero on the initialization
    float epsilon = 0.000001;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
        
        float rho = measurement_pack.raw_measurements_[0]; // range
        float phi = measurement_pack.raw_measurements_[1]; // bearing
        
        // float rho_dot = measurement_pack.raw_measurements_[2]; // rho speed
        
        float x = rho * cos(phi);
        float y = rho * sin(phi);
        float vx = 0; // rho_dot * cos(phi);
        float vy = 0; // rho_dot * sin(phi);
        
        if (fabs(x) < epsilon or fabs(x) < epsilon){
            x = 0;
            y = 0;
        }
        
        ekf_.x_ << x, y, vx, vy;
        
        
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
        // set the state with the initial location and zero velocity
        ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
        if (fabs(ekf_.x_[0]) < epsilon or fabs(ekf_.x_[1]) < epsilon){
            ekf_.x_ << 1, 1, 1, 1;
        }
    }
      
      
      
      
      // Initial covariance matrix
      
      
      previous_timestamp_ = measurement_pack.timestamp_;
      
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
    
    //compute the time elapsed between the current and previous measurements
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;    //dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

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
    
    ekf_.F_ <<  1, 0, dt, 0,
                0, 1, 0, dt,
                0, 0, 1, 0,
                0, 0, 0, 1;
    
    
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;
    
    //set the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
                0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
                dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
                0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
    
    ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
      
      
      Hj_ << tools.CalculateJacobian(ekf_.x_);
      ekf_.H_ = Hj_;
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
      
  } else {
    // Laser updates
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
    if (measurement_pack.sensor_type_ == measurement_pack.RADAR) {
        
        cout << "measure type " << measurement_pack.sensor_type_ << endl;
        cout << "measure = " << measurement_pack.raw_measurements_ << endl;
        cout << "x_ = " << ekf_.x_ << endl;
//        cout << "P_ = " << ekf_.P_ << endl;
    }
    
}

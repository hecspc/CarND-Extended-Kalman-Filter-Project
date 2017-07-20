#include "kalman_filter.h"
#include <iostream>

using namespace std;
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
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    calculateKF(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    // Recalculate x object state to rho, theta, rho_dot coordinates
    double px = x_[0];
    double py = x_[1];
    double vx = x_[2];
    double vy = x_[3];
    double epsilon = 0.0001;
    
    double rho = sqrt(px * px + py * py);
    double phi;
    double rho_dot;
    if (rho < epsilon) {
        phi = 0;
        rho_dot = 0;
    }else {
        phi = atan2(py, px);
        rho_dot = (px * vx + py * vy) / rho;
    }
    
    VectorXd h = VectorXd(3); // h(x_)
    h << rho, phi, rho_dot;
    
    VectorXd y = z - h;
//    if (y(1) > M_PI) {
//        cout << "GREATER THAN 2 PI" << endl;
//        y << y(0), y(1) - M_PI, y(2);
//    }else if (y(1) < -M_PI) {
//        y << y(0), y(1) + M_PI, y(2);
//    }
    
    if(y(1) > M_PI)
    {
        while(y(1) > M_PI)
        {
            std::cout << "y(1) > M_PI\n";
            y(1) -= 2*M_PI;
        }
    }
    else if (y(1) < -M_PI)
    {
        while (y(1) < -M_PI)
        {
            std::cout << "y(1) < -M_PI\n";
            y(1) += 2 * M_PI;
        }
    }
    cout << "y: " << y << endl;
    calculateKF(y);
}

void KalmanFilter::calculateKF(const VectorXd &y) {
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

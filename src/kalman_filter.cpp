#include "kalman_filter.h"
#include <math.h>       /* atan2 */

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define M_PI 3.14159265358979323846

/*
 * defines the predict function, the update function for lidar, and the update function for radar */
 
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  /**
   * TODO: predict the state
   */
    x_ =F_ * x_;
    MatrixXd Ft_ = F_.transpose();
    P_ = F_ * P_* Ft_ + Q_;
    
    
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
    
    VectorXd z_pred_= H_* x_;
    VectorXd y_= z - z_pred_;
    
    Upadatesame(y_);
    
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

    //Instead, for extended Kalman filters, we'll use the h function directly to map predicted locations x'x from Cartesian to polar coordinates.
    
    double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    //double phi = atan(x_(1) / x_(0));
    double phi = atan2(x_(1) , x_(0)); // Principal arc tangent of y/x, in the interval [-pi,+pi] radians.
    double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
    
    VectorXd hx_ = VectorXd(3);
    hx_ << rho, phi, rho_dot;
    
    VectorXd y_= z - hx_;

    //normallizing the angle
    while (y_(1)>M_PI){
      y_(1) -= 2* M_PI;
    }

    while (y_(1)<-M_PI){
      y_(1) += 2* M_PI;
    }
    
    
    Upadatesame(y_);

}

void KalmanFilter::Upadatesame(const VectorXd &y) {
    /**
         same equations in both Laser and Radar mesaurement update step
     */
    
    //calculation of S and K
    MatrixXd Ht_= H_.transpose();
    MatrixXd S_ = H_ * P_ * Ht_ + R_;
    MatrixXd Si_ = S_.inverse();
    MatrixXd PHt_ =P_ * Ht_;
    MatrixXd K_ = PHt_ * Si_;
    
    //new estimate
    x_ = x_ + (K_ * y);
    long x_size_ = x_.size();
    MatrixXd I_ =MatrixXd:: Identity(x_size_,x_size_ );
    
    P_ =(I_ - K_*H_) * P_;
}


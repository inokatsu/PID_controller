#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
//  Kp = Kp_;
//  Ki = Ki_;
//  Kd = Kd_;
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;
}

void PID::UpdateError(double cte) {
  
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  
  return -Kp * p_error - Kd * d_error - Ki * i_error;
}

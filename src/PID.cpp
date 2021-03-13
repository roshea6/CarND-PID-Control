#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients and errors
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  p_error = 0;
  i_error = 0;
  d_error = 0;
  
  prev_cte = 0;

}

// Update the error values for each of the components based on the standard PID equations
void PID::UpdateError(double cte) {
 
  p_error = cte;
  d_error = cte - prev_cte;
  i_error += cte;
  
  // Update the previous cte so it can be used next time
  prev_cte = cte;

}

// Calculate the totaly error with the standard PID formula
double PID::TotalError() {

  double total_error = -Kp*p_error - Kd*d_error - Ki*i_error;
  
  return total_error;  
}
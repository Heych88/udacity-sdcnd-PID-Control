#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double K_p, double K_i, double K_d) {
  Kp = K_p;
  Ki = K_i;
  Kd = K_d;
      
  i_total_error = 0;
  d_prev_error = 0; 
  
  output = 0;
}

void PID::UpdateError(double cte) {
  //double p_shift, i_shift, d_shift;
  
  // proportion update
  p_error = Kp * cte;
  
  // integral update
  i_total_error += cte;
  i_error = Ki * i_total_error;
 
  // derivitve update
  if(d_prev_error == 0){
    d_error = 0;
  } else {
    d_error = Kd * (cte - d_prev_error);
  }
  d_prev_error = cte; // store the current error for the next update
  
  output = p_error + i_error + d_error;
}

double PID::TotalError() {
  return i_total_error;
}


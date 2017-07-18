#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double K_p, double K_i, double K_d, double out_max, double out_min) {
  Kp = K_p;
  Ki = K_i;
  Kd = K_d;
  max_out = out_max;
  min_out = out_min;

  i_total_error = 0;
  d_prev_error = 0; 
  
  output = 0;
  
  use_twiddle = true;
  best_twiddle = 0;
  current_twiddle = 0;
  current_param = 0;
  params[0] = 0;
  params[1] = 0;
  params[2] = 0;
  delta_params[0] = 0.1;
  delta_params[1] = 0.5;
  delta_params[2] = 0.01;
  params_thresh[0] = 0.01;
  params_thresh[1] = 0.01;
  params_thresh[2] = 0.0001;
}

void PID::UpdateError(double cte) {
  //double p_shift, i_shift, d_shift;
  
  // proportion update
  p_error = Kp * cte;
  
  // integral update
  i_total_error += cte;
  i_error = Ki * i_total_error;
 
  // derivative update
  if(d_prev_error == 0){
    d_error = 0;
  } else {
    d_error = Kd * (cte - d_prev_error);
  }
  d_prev_error = cte; // store the current error for the next update
  
  output = p_error + i_error + d_error;
  if(output > max_out) output = max_out;
  else if(output < min_out) output = min_out;
}

double PID::TotalError() {
  return i_total_error;
}


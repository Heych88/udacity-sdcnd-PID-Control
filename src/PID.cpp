#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(const double K_p, const double K_i, const double K_d, const double out_max, const double out_min, const bool twiddle) {
  Kp = K_p;
  Ki = K_i;
  Kd = K_d;
  max_out = out_max;
  min_out = out_min;

  i_total_error = 0;
  d_prev_error = 0; 
  
  output = 0;
  
  use_twiddle = twiddle; 
  first_run = true;
  best_twiddle = 0;
  current_twiddle = 0;
  current_param = 0;
  
  // initial parameter values
  params[0] = 0; // Kp
  params[1] = 0; // Ki
  params[2] = 0; //Kd
  // initial change in parameter values
  delta_params[0] = 0.1; // Kp delta
  delta_params[1] = 0.01; // Ki delta
  delta_params[2] = 2; // Kd delta
  params_thresh[0] = 0.01; // Kp max delta value considered as a correct value
  params_thresh[1] = 0.0001;
  params_thresh[2] = 0.01;
}

// PID system controller
// cte = desired value - current value

void PID::UpdateError(const double cte) {
  
  // proportion update
  p_error = Kp * cte;
  
  // integral update
  // sums the system error to kept track of system drift
  i_total_error += cte; 
  i_error = Ki * i_total_error;
 
  // derivative update
  // compare the change between the current and past errors
  if(d_prev_error == 0){
    // check if it is the first controller pass
    d_error = 0;
  } else {
    d_error = Kd * (cte - d_prev_error); 
  }
  d_prev_error = cte; // store the current error for the next update
  
  output = p_error + i_error + d_error;
  
  // clip the output to be between the min and max thresholds
  if(output > max_out) output = max_out;
  else if(output < min_out) output = min_out;
}

double PID::TotalError() {
  return i_total_error; 
}


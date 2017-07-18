#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  
  double d_prev_error;
  double i_total_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  /*
   * Gradient descent 
   */  
  int best_twiddle; // stores the best parameter twiddle error so far
  int current_twiddle; // stores the current runs error value
  int current_param; // keeps track of which twiddle controller parameter is being tuned
  double params[3]={}; // array containing the initial parameter values
  double delta_params[3]={}; // array containing the initial delta values 
  double params_thresh[3]={}; // array containing the acceptable thresholds for parameter checks
  bool use_twiddle; // enables twiddle parameter tuning when set to true
  bool first_run; // keeps track of the first twiddle run
  
  /*
   * Final controller output
   */
  double output; // store the output of the controller 
  double max_out; // max clipped output value of the controller
  double min_out; // min clipped output value

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(const double K_p, const double K_i, const double K_d, const double out_max=1, const double out_min=-1, const bool twiddle=false);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(const double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */

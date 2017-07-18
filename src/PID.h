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
  //double params[3];
  //double delta_params[3];
  
  //int best_error;
  //int current_error;
  
  //bool use_twiddle = true;
  //bool start_twiddle = true;
  
  int best_twiddle;
  int current_twiddle;
  int current_param;
  double params[3]={};
  double delta_params[3]={};
  double params_thresh[3]={};
  bool use_twiddle;
  
  /*
   * Final controller output
   */
  double output;
  double max_out;
  double min_out;

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
  void Init(double Kp, double Ki, double Kd, double out_max=1, double out_min=-1);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */

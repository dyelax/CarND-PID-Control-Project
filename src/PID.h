#ifndef PID_H
#define PID_H
#include <iostream>
#include <float.h>
#include <uWS/uWS.h>

class PID {
public:
  /*
  * Errors
  */
  double error_terms [3];
  
  double err_squared_sum; // To calculate MSE for Twiddle
  double err_sum;
  double err_prev;
  bool err_prev_initialized;

  /*
  * Coefficients
  */
  double coeffs[3];
  
  /*
   * Twiddle vars
   */
  double coeffs_d[3];
  int step_num;
  int twiddle_iterations;
  int twiddling_coeff_i;
  bool up_run;
  float best_err;
  
  int MAX_STEPS;
  float THRESHOLD;
  float SCALING;
  bool TWIDDLE;
  
  /*
  * Constructor
  */
  PID(bool should_twiddle, double Kp, double Kd, double Ki);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double err);
  
  /*
   * Get the control estimate for an error.
   */
  double GetAlpha(double err, double min, double max);
  
  /*
   * Get the mean squared error for a Twiddle iteration.
   */
  double GetMSE(int num_steps);

  /*
   * Reset the simulator for another run
   */
  void Reset(uWS::WebSocket<uWS::SERVER> ws, bool increment_coeff_i);
  
  /*
   * Optimize parameters with Twiddle
   */
  void TwiddleStep(uWS::WebSocket<uWS::SERVER> ws);
};

#endif /* PID_H */

#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(bool should_twiddle, double Kp, double Kd, double Ki) {
  coeffs[0] = Kp;
  coeffs[1] = Kd;
  coeffs[2] = Ki;
  
  coeffs_d[0] = 1;
  coeffs_d[1] = 1;
  coeffs_d[2] = 1;
  
  twiddle_iterations = 0;
  twiddling_coeff_i = 0;
  up_run = false;
  
  MAX_STEPS = 1000;
  THRESHOLD = 1e-3;
  SCALING = 0.1;
  TWIDDLE = should_twiddle;
  best_err = FLT_MAX;
}

PID::~PID() {}

void PID::UpdateError(double err) {
  if (!err_prev_initialized) {
    err_prev = err;
    err_prev_initialized = true;
  }
  
  error_terms[0] = -coeffs[0] * err;
  error_terms[1] = -coeffs[1] * (err - err_prev);
  error_terms[2] = -coeffs[2] * err_sum;
  
  err_prev = err;
  err_sum += err;
  err_squared_sum += err * err;
}

double PID::GetAlpha(double err, double min, double max) {
  UpdateError(err);
  double alpha = error_terms[0] + error_terms[1] + error_terms[2];
  
//  cout << "Alpha pre-clamp: " << alpha << endl;

  if (alpha > max) {
    alpha = max;
  } else if (alpha < min) {
    alpha = min;
  }
  
  return alpha;
}

double PID::GetMSE(int num_steps) {
  return err_squared_sum / num_steps;
}

void PID::Reset(uWS::WebSocket<uWS::SERVER> ws, bool increment_coeff_i) {
  err_sum = 0;
  err_squared_sum = 0;
  err_prev_initialized = false;
  
  step_num = 0;
  twiddle_iterations++;
  
  if (increment_coeff_i) {
    twiddling_coeff_i = (twiddling_coeff_i + 1) % 3;
    
    coeffs[twiddling_coeff_i] += coeffs_d[twiddling_coeff_i];
    up_run = true;
  } else {
    up_run = false;
  }
  
  cout << "New params" << endl;
  cout << "Kp: " << coeffs[0] << endl;
  cout << "Kd: " << coeffs[1] << endl;
  cout << "Ki: " << coeffs[2] << endl;
  
  // Reset simulator
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

void PID::TwiddleStep(uWS::WebSocket<uWS::SERVER> ws) {
//  cout << "step_num: " << step_num << endl;
  step_num++;
  
  if (step_num >= MAX_STEPS) {
    // init twiddle after 1 run
    if (twiddle_iterations == 0) {
      best_err = GetMSE(step_num);
      
      Reset(ws, true);
      return;
    }
    
    double mse = GetMSE(step_num);
    
    cout << "MSE : " << mse << endl;
    cout << "Best: " << best_err << endl;
    
    if (mse < THRESHOLD) {
      ws.close();
      cout << "Final params" << endl;
      cout << "Kp: " << coeffs[0] << endl;
      cout << "Kd: " << coeffs[1] << endl;
      cout << "Ki: " << coeffs[2] << endl;
    } else {
      if (mse < best_err) {
        best_err = mse;
        coeffs_d[twiddling_coeff_i] *= (1 + SCALING);
        
        Reset(ws, true);
        return;
      } else {
        if (up_run) {
          coeffs[twiddling_coeff_i] -= 2 * coeffs_d[twiddling_coeff_i];
          
          Reset(ws, false);
          return;
        } else {
          coeffs[twiddling_coeff_i] += coeffs_d[twiddling_coeff_i];
          coeffs_d[twiddling_coeff_i] *= (1 - SCALING);
          
          Reset(ws, true);
          return;
        }
      }
    }
  }
}


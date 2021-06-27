#include "PID.h"

#include <numeric>
#include <vector>

#include "iostream"

using std::accumulate;
using std::vector;

/**
 * TODO: Complete the PID class. You may add any additional desired
 * functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}

void PID::twiddle(double cte) {
  std::cout << " Iteration " << epoc << " best error " << best_err << std::endl;

  double params[] = {Kp, Kd};
  // learning rate
  double lr = 0.1;
  double dp[] = {lr * d_error, lr * d_error};
  std::cout << " cte - p_error " << d_error << std::endl;
  if (fabs(TotalError()) > tol) {
    params[idx] += dp[idx];
    if (fabs(cte) < fabs(best_err)) {
      best_err = cte;
      dp[idx] *= 1.1;
    } else {
      params[idx] -= 2 * dp[idx];
      if (fabs(cte) < fabs(best_err)) {
        best_err = cte;
        dp[idx] *= 1.1;
      } else {
        params[idx] += dp[idx];
        dp[idx] *= 0.9;
        // epoc step
        if (epoc % 3 == 0) {
          best_err = std::numeric_limits<double>::max();
          idx = (idx + 1) % 2;
        }
      }
    }
    epoc += 1;
  }

  Kp = params[0];
  Kd = params[1];

  std::cout << " Kp: " << params[0] << "  Ki: " << Ki << "  Kd: " << params[1]
            << std::endl;
}

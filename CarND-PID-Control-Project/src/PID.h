#ifndef PID_H
#define PID_H
#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  std::vector<double> store_cte;
  double prev_cte;
  double total_error;
  double best_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  int param_index;
  double changes[3];
  int count;
  bool condition;
  int n_step;

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
  void Init(double Kp, double Ki, double Kd, int n_step);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  double SteerAngle();

  void Twiddle();

};

#endif /* PID_H */

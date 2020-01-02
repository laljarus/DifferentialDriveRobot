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
  double prev_d_error;


  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;
  double Tf;

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
  void Init(double Kp, double Ki, double Kd,double N);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte,double dt);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */

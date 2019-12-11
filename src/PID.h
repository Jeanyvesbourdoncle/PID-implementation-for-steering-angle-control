//============================================================================
// Name        : PID.h
// Author      : Jean-Yves Bourdoncle
// Version     : v1.0
// Date		   : 23/06/2019
//============================================================================


#ifndef PID_H
#define PID_H

#include <vector>


class PID {
 public:
  // Constructor
  double p_error;
  double i_error;
  double d_error;

  //PID Coefficients
  double Kp;
  double Ki;
  double Kd;

  int cycle;

  PID();
  //Destructor.
  virtual ~PID();

  //Initialize PID : @param (Kp_, Ki_, Kd_) The initial PID coefficients
  void Init(double Kp, double Ki, double Kd);

  //Update the PID error variables given cross track error.@param cte The current cross track error
  void UpdateError(double cte);

  // Calculate the total PID error.@output The total PID error
  double TotalError();

  // PID Errors
};

#endif  // PID_H

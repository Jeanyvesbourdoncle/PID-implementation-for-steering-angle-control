//============================================================================
// Name        : PID.cpp
// Author      : Jean-Yves Bourdoncle
// Version     : v1.0
// Date		   : 23/06/2019
// Description : Hyperparameters Initialization : Gain Initialization and Error initialization
//				 Update Error : based on the Cross Track Error CTE
//				 Total Error Initialization : Calculation of the alpha with the Kp,Kd,Ki hyperparameters
//============================================================================
#include <vector>
#include <iostream>
#include <cmath>
#include "PID.h"

PID::PID() {}
PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  //Initialize PID coefficients (and errors, if needed)
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  cycle=0; // initialization for the first cycle
}

void PID::UpdateError(double cte) {
  //Update PID errors based on cte.
  d_error = cte-p_error; // derivate
  p_error = cte; // proportional
  i_error += cte; // integrale

  cycle=cycle+1; // cycle incrementation
}

double PID::TotalError( ) {
  //Calculate and return the total error after the hyperparameters tuning
  return -Kp * p_error  -Kd * d_error -Ki * i_error;
}

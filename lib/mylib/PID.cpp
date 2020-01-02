#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_in, double Ki_in, double Kd_in, double Tf_in) {
	Kp = Kp_in;
	Ki = Ki_in;
	Kd = Kd_in;
	Tf = Tf_in; // time constant for low pass filter for the derivative term
	p_error = 0;
	i_error = 0;
	d_error = 0;
	prev_d_error = 0;
}

void PID::UpdateError(double cte, double dt) {

	// p_error = previous cte

	if (Tf == 0){
		d_error = Kd*(cte-p_error)/dt;
	}else {
		d_error = (cte*Kd - p_error*Kd + prev_d_error*Tf)/(Tf+dt);
		prev_d_error = d_error;
	}

	p_error = cte;
	i_error += cte*dt;

}

double PID::TotalError() {

	double value;

	value = Kp*p_error + Ki*i_error + d_error;

	return value;

}

#include "PID.h"

PID_Controller::PID_Controller(double Kp, double Ki, double Kd) 
{
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    int_error_ = prev_error_ = 0.0;
}

double PID_Controller::TrackError(double cur_error, double dt) 
{
    // ignore if elapsed time is too small
    if (dt < 1e-3)
        return 0.0;

    // estimate error derivative
    double der_error = (cur_error - prev_error_) / dt;
    
    // update total error integral
    int_error_ += cur_error*dt;

    // correction = PID weighted sum
    double correction = -Kp_*cur_error - Ki_*int_error_ - Kd_*der_error; 

    prev_error_ = cur_error;
    return correction;
}

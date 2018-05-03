#ifndef PID_H
#define PID_H

class PID_Controller 
{
public:
    //
    // Initialize with user-defined control coefficients:
    //   Kp = proportional gain
    //   Ki = integral gain
    //   Kd = derivative gain
    // 
    PID_Controller(double Kp, double Ki, double Kd);

    // Compute new correction, based on current error and elapsed time
    double TrackError(double cur_error, double dt);

private:
    // PID gain coefficients
    double Kp_, Ki_, Kd_; 

    double int_error_;  // running estimate of error integral
    double prev_error_; // previous error value
};

#endif /* PID_H */

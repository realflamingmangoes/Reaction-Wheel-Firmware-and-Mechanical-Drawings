/** @file Controller.h
 *  This file contains the Controller class which currently contains an integrator
 *  that calculates speed values from commanded torques.
*/

#pragma once

class Controller 
{
    protected:
    
        const float J;                  // moment of inertia
        unsigned long last_update_ms;   // last time calculate_omega() was called (ms)
        float omega_rad_s;              // wheel speed in rad/s for integration
        float kp;
        float kd;
        float alpha; // low pass filter constant for derivative term
        float speed_last;
        bool initialized;
        float d_speed_filt;
        float u = 0.0f;
        float error_last;
        
    public:
        
        // These functions are commented in Controller.cpp
        Controller(void);
        float calculate_omega(float torque_cmd_);
        float PD(float speed_cmd, float speed_meas, float dt_s);
        void update_gains(float kp_new, float kd_new);
        float get_kp(void) const;
        float get_kd(void) const;
};


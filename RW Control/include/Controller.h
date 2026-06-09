/** @file Controller.h
 *  This file contains the Controller class which currently contains an integrator
 *  that calculates speed values from commanded torques.
*/

#pragma once

/** This class is used to calculate speed commands for the state machine */
class Controller 
{
    protected:
    
        const float J;                  // moment of inertia
        unsigned long last_update_ms;   // last time calculate_omega() was called (ms)
        float omega_rad_s;              // wheel speed in rad/s for integration
        
    public:
        
        // These functions are commented in Controller.cpp
        Controller(void);
        float calculate_omega(float torque_cmd_);
};


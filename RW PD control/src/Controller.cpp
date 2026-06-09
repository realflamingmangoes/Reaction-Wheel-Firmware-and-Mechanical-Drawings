/** @file Controller.cpp
 *  This file contains the Controller class which currently contains an integrator
 *  that calculates speed values from commanded torques.
*/

#include <Arduino.h>
#include "Controller.h"
#include "Shares.h"
#include <PrintStream.h>

/** @brief Constructor which sets up a Controller class defining a constant flywheel 
 *  moment of inertia and initializing values for the integrator and PD controller.
 */
Controller::Controller(void)
    : J(0.001716) // kg * m^2, moment of inertia for the motor and load
{
    last_update_ms = 0;
    omega_rad_s    = 0.0f;
    kp = 0.010f;
    kd = 0.005f;
    // kd = 0.000f; // set derivative gain to zero for now since we haven't tuned it yet and the low pass filter is pretty aggressive
    alpha = 0.95f; // low pass filter constant for derivative term
    error_last = 0.0f;
    speed_last = 0.0f;
    d_speed_filt = 0.0f;
    initialized = false;
}

/** @brief This function integrates torque to get speed.
 * 
 *  @details This function uses a forward Euler integrator to integrate a commanded torque input to 
 *  convert it to a speed.
 * 
 *  @param torque_cmd_ The torque to integrate into a speed. The time used is the time between the 
 *  last command and this command.
 * 
 *  @return The calculated speed command in RPM, which is then passed to the state machine to command the motor.
 */
float Controller::calculate_omega(float torque_cmd_)
{

    // These variables are declared inside the calculate_omega command so we can keep them separate from the PID task 
    // we will implement later
    // set now_ms as the amount of milliseconds since system boot
    unsigned long now_ms = millis();

    // declare dt_s
    float dt_s;

    dt_s = (now_ms - last_update_ms) / 1000.0f;
    if (dt_s <= 0.0f) // Guard against millis() wrap or weirdness
    {
        dt_s = 0.001f;
    }
    else if (dt_s >= 0.3f) // Clamp dt_s at 0.3 second so incase a command drops, it doesn't calculate an unreasonable speed
    {
        dt_s = 0.3f;
    }
    
    // set last_update_ms to now_ms for the next iteration
    last_update_ms = now_ms;

    // Calculate angular acceleration [rad/s^2]
    float alpha = torque_cmd_ / J;

    // Forward Euler Integrator
    // Integrate to get new angular speed [rad/s]
    float torque_add = alpha * dt_s;
    // Add the actual speed converted to rad/s
    omega_rad_s = torque_add + last_speed_cmd.get() * (2.0f * PI / 60.0f);

    last_speed_cmd.put(omega_rad_s * (60.0f / (2.0f * PI))); // Update last_speed_cmd for the next iteration

    // Clamp omega to the physical limit of the BLDC motor (<2760 RPM)
    const float omega_max_rad_s = 2.0f * PI * 2500.0f / 60.0f; // ~2500 RPM
    if (omega_rad_s > omega_max_rad_s)  omega_rad_s = omega_max_rad_s;
    if (omega_rad_s < -omega_max_rad_s) omega_rad_s = -omega_max_rad_s;

    // Convert rad/s → RPM
    float omega_rpm = omega_rad_s * (60.0f / (2.0f * PI));

    return omega_rpm;
}

/** @brief This function implements a PD controller.
 * 
 *  @details This function uses a PD controller to generate control commands for the internal control loop.
 * 
 *  @param speed_cmd The commanded speed.
 *  @param speed_meas The measured speed.
 *  @param dt_s The time step for the derivative calculation.
 * 
 *  @return The calculated control command.
 */
float Controller::PD(float speed_cmd, float speed_meas, float dt_s)
{

    float cmd_abs  = fabsf(speed_cmd);
    float meas_abs = fabsf(speed_meas);

    float error_now = cmd_abs - meas_abs;

    if (!initialized)
    {
        error_last = error_now;
        speed_last = meas_abs;
        d_speed_filt = 0.0f;
        initialized = true;
    }

    // Derivative of measured speed, not derivative of error.
    // This avoids huge derivative kick when speed_cmd changes.
    float d_speed = (meas_abs - speed_last) / dt_s;

    // Low-pass filter
    d_speed_filt = alpha * d_speed_filt + (1.0f - alpha) * d_speed;

    speed_last = meas_abs;
    error_last = error_now;

    float p_term = kp * error_now;
    float d_term = kd * d_speed_filt;

    float u_norm = p_term - d_term;

    if (u_norm < 0.0f) u_norm = 0.0f;
    if (u_norm > 1.0f) u_norm = 1.0f;

    return u_norm * 4095.0f;
}

/** @brief If called, this function updates the kp and kd class variables.
 *  * 
 *  @param kp_new The new kp value.
 *  @param kd_new The new kd value.
 */
void Controller::update_gains(float kp_new, float kd_new)
{
    kp = kp_new;
    kd = kd_new;
}

/** @brief If called, this function returns the current kp value.
 *  * 
 *  @return The current kp value.
 */
float Controller::get_kp(void) const
{
    return kp;
}   

/** @brief If called, this function returns the current kd value.
 *  * 
 *  @return The current kd value.
 */
float Controller::get_kd(void) const
{
    return kd;
}   
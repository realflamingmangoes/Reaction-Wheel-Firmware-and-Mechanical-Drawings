/** @file Controller.cpp
 *  This file contains the Controller class which currently contains an integrator
 *  that calculates speed values from commanded torques.
*/

#include <Arduino.h>
#include "Controller.h"
#include "Shares.h"
#include <PrintStream.h>

/** @brief Constructor which sets up a Controller class defining a constant flywheel 
 *  moment of inertia and initializing values for the integrator.
 */
Controller::Controller(void)
    : J(0.001716) // kg * m^2, moment of inertia for the motor and load
{
    last_update_ms = 0;
    float omega_rad_s    = 0.0f;
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

/** @file Shares.h
 *  This file contains extern declarations of shares and queues used for wireless control of the 
 *  motor and live plotting in the GUI.
 */

#pragma once

#include <PrintStream.h>  
#include "taskqueue.h"
#include "taskshare.h"

// A share which stores the fault state of the reaction wheel
extern Share<uint32_t> rw_fault_flags;

// A queue which holds torque command values from the webserver and passes them to the calcSetpoint task
extern Queue<float> torque_cmd;

// A share which holds speed command values from the webserver and passes them to the speedControl state machine
extern Share<float> speed_cmd;
extern Share<float> last_speed_cmd; // last speed command for integration in Controller

// A share which populates using an ISR and holds the current speed of the motor
extern Share<float> speed_actual;

// A queue which uses an ISR to trigger the readActual task and calculate the motor speed by reading the square wave 
// frequency on the FGOUT pin
extern Queue<uint32_t> edge_time;

extern Queue<float> edge_count;

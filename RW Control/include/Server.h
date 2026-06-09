/** @file Server.h 
 *  This file contains methods and tasks needed to setup and run a web server hotspot
 *  from an ESP32 microcontroller.
*/

#pragma once

/** These methods and tasks, along with other internals, are commented in Server.cpp */
void setup_wifi(void);
void task_webserver(void* p_params);


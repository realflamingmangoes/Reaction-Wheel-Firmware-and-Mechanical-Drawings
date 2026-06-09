/** @file main.cpp
 *  This program creates an interface to actuate a brushless DC motor as part of a momentum exchange device meant to 
 *  simulate attitude control maneuvers on a frictionless test platform. It allows for speed and torque control, and 
 *  has a simple web interface to wirelessly command the BLDC motor to any speed +/- 10 RPM.
 * 
 *  @author Neil Bedagkar with help from Bricen Rigby. 
 * 
 *  The code was written using help from ChatGPT 5.1 and Claude Sonnet 4.5. Most of the HTML script was written by AI. 
 *  The original code to setup a hotspot on the ESP32 was developed by John Ridgely, taken from an in-class example.
 * 
 *  In addition, the files @c taskshare.h and @c taskqueue.h are used; these files are in
 *  the ME507 support package at @c https://github.com/spluttflob/ME507-Support
 *  from which code can be used in PlatformIO or Arduino using the link which 
 *  is accessed using the green @b Code button on the GitHub page. 
 *  @c PrintStream.h is used to support these two files, which is in the package at
 *  @c https://github.com/spluttflob/Arduino-PrintStream.git.
 * 
 *  @date 12/9/2025
 * 
 *  @copyright (c) 2025 by Neil Bedagkar and Bricen Rigby, released under GPL 3.0
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Shares.h"
#include "Driver.h"
#include "Controller.h"
#include "taskshare.h"
#include "taskqueue.h"
#include <PrintStream.h>
#include "Server.h"
#include "driver/pcnt.h"

//================= CONFIG =================
#ifndef RW_I2C_ADDR
#define RW_I2C_ADDR 0x22   // unique per wheel (set per ESP32 build)
#endif

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ    400000

#define MOTOR_CTRL_HZ 500
//==========================================

// Enable to print raw RPM/pulse diagnostic info to Serial
#define DEBUG_RPM 1

//------------- I2C register map -------------
static constexpr uint8_t REG_TORQUE_CMD = 0x00; // write 4 bytes float torque [N*m]
static constexpr uint8_t REG_RPM_AND_FAULTS = 0x04; // read 8 bytes: float rpm + uint32 faults
static constexpr uint8_t REG_FAULTS = 0x08; // read 4 bytes: uint32 faults

// I2C "current register" pointer set by the last master write
static volatile uint8_t reg_ptr = 0;

static const pcnt_unit_t PCNT_UNIT_USED = PCNT_UNIT_0;
static const pcnt_channel_t PCNT_CH_USED = PCNT_CHANNEL_0;

static gpio_num_t PULSE_GPIO = (gpio_num_t)25;   // set to your GPIO

// Create motor driver object
Driver Peripheral;

// Create motor controller object
Controller Controller_1;

/** @brief Function which returns the sign of the input
 * 
 *  @param x Floating point integer
 * 
 *  @return Sign of x
 */
inline int sign(float x) {
    if (x==0) {return 0;}
    else if (x>0) {return 1;}
    else if (x<0) {return -1;}
    else {return 0;}
}

//--------------- I2C Helpers ---------------
static inline void wireWriteU32(uint32_t v)
{
    Wire.write(reinterpret_cast<uint8_t*>(&v), sizeof(v));
}

static inline void wireWriteF32(float v)
{
    Wire.write(reinterpret_cast<uint8_t*>(&v), sizeof(v));
}

//================ I2C CALLBACKS ================
// Master write pattern supported:
//  1) write 1 byte register pointer
//  2) optionally followed by payload
//
// Torque write:
//  write: [REG_TORQUE_CMD][4 bytes float]
void onI2CReceive(int nbytes) 
{ 
    if (nbytes <= 0) return; 
    reg_ptr = Wire.read(); 
    nbytes--; 
    if (reg_ptr == REG_TORQUE_CMD && nbytes >= 4) 
    { uint8_t buf[4]; for (int i = 0; i < 4; i++) 
        buf[i] = Wire.read(); 
        float tq; memcpy(&tq, buf, sizeof(float)); 
        //Serial.printf("I2C tq = %.6f\n", tq); 
        torque_cmd.put(tq); 
        // Serial.printf("Torque command received: %.3f N*m\n", tq);
    } 
    // Drain anything else 
    while (Wire.available()) (void)Wire.read(); 
}

// Master read pattern supported:
//  write: [REG_xx]      (no payload)
//  read:  N bytes
void onI2CRequest()
{
    if (reg_ptr == REG_RPM_AND_FAULTS)
    {
        const float rpm = speed_actual.get();
        const uint32_t faults = rw_fault_flags.get();
        wireWriteF32(rpm);
        wireWriteU32(faults);
        return;
    }

    if (reg_ptr == REG_FAULTS)
    {
        const uint32_t faults = rw_fault_flags.get();
        wireWriteU32(faults);
        return;
    }

    // Default: return zero (4 bytes)
    float zero = 0.0f;
    wireWriteF32(zero);
}

/** @brief Task which live tunes the controller
 * 
 *  @details This task monitors the kp and kd queues for updates and applies them to the controller. 
 *  The kp and kd queues are filled from the webserver interface, where gains are updated by the user to 
 *  automate the tuning process. Because it is watching multiple queues, it polls at a fixed rate rather 
 *  than blocking on a single queue to allow the gains to be tuned independently.
*/
void task_gainWatchdog(void* parameters)
{
    float kp_current = Controller_1.get_kp();
    float kd_current = Controller_1.get_kd();

    while (true)
    {
        while (kp.is_empty() && kd.is_empty())
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        if (!kp.is_empty())
        {
            kp_current = kp.get();
            Serial.printf("Queued Kp update: %.6f\n", kp_current);
        }

        if (!kd.is_empty())
        {
            kd_current = kd.get();
            Serial.printf("Queued Kd update: %.6f\n", kd_current);
        }

        Controller_1.update_gains(kp_current, kd_current);
        Serial.printf("Gains updated: Kp=%.3f, Kd=%.3f\n", kp_current, kd_current);
    }
}

/** @brief Task which reads the speed of the motor
 * 
 *  @details The BLDC motor has Hall sensors which output a square wave at the electrical
 *  frequency of the motor. The DRV8308 chip outputs a square wave of this frequency, which is
 *  read by an ISR that puts a timestamp in the edge_time queue each time there is a rising edge. 
 *  This task gets that timestamp, compares it to the previous timestamp, calculates the frequency 
 *  of the motor in RPM and places that in the speed_actual queue. This task runs at a fixed rate 
 *  of 100 Hz to avoid aperiodicity in the control architecture. 
*/
void task_readActual(void* parameters) 
{
    
    uint32_t last_time = micros();  // initialize the last_time to the time at setup
    uint32_t current_time = 0;      // initialize current_time to zero    
    float frequency = 0.0;          // initialize electrical frequency to zero
    float rpm = 0.0;                // initialize RPM to zero
    uint32_t dt_us = 0;             // initialize timestamp delta to zero
    bool direction = LOW;           // initialize direction boolean to low
    float edges = 10;
    
    static float w[5] = {0,0,0,0,0};
    static int wi = 0;
    float rpm_filt_prev = 0.0f;

    while (true) 
    {
        current_time = edge_time.get();     // Task only runs once there is a value in edge_time
        pcnt_counter_clear(PCNT_UNIT_0);  // clear counter for next measurement

        dt_us = current_time - last_time;   // calculate dt between rising edges
        last_time = current_time;           // set last_time to the current time after dt calculation

        // prevent dividing by zero error
        if (dt_us == 0)                     
        {
          continue;
        }

        // change number of edges counted based on the speed of the motor to improve accuracy at low speeds (tuning parameter)
        if (!edge_count.is_empty())
        {
            edges = edge_count.get();
            Peripheral.pcnt_set_threshold((uint16_t)edges);
        }

        // Calculate electrical frequency of motor by inverting dt_us and converting to seconds (Hz)
        frequency = edges / (2.0f * ((float)dt_us) / 1000000); 

        // Use the direction of current motor spin to calculate positive or negative rpm
        direction = Peripheral.get_dir(); 
        float rpm_magnitude = frequency * 15.0f;
        if (direction == LOW) {rpm = rpm_magnitude;}
        else {rpm = -rpm_magnitude;}

        w[wi] = rpm; wi = (wi + 1) % 5;

        // Copy + sort for median
        float s[5];
        for (int i=0;i<5;i++) s[i] = w[i];
        for (int i=0;i<5;i++)
        for (int j=i+1;j<5;j++)
            if (s[j] < s[i]) { float tmp=s[i]; s[i]=s[j]; s[j]=tmp; }

        float rpm_med = s[2]; // median of 5

        if (rpm_med == 0.0f) rpm_med = rpm; // if median is zero, use raw value (prevents lockup at low speeds)

        // 2) Rate limit the output (slew limit)
        static float rpm_filt = 0.0f;

        // Estimate dt (seconds) from dt_us you already computed
        float dt_s_local = (float)dt_us * 1e-6f;
        if (dt_s_local < 0.001f) dt_s_local = 0.001f;
        if (dt_s_local > 0.2f)   dt_s_local = 0.2f;

        // Pick a physically plausible max accel in RPM/s (tune this!)
        const float max_slew_rpm_per_s = 3000.0f;  // example
        float max_step = max_slew_rpm_per_s * dt_s_local;

        float err = rpm_med - rpm_filt;
        if (err >  max_step) err =  max_step;
        if (err < -max_step) err = -max_step;

        rpm_filt += err;

        speed_actual.put(rpm_filt);
        rpm_filt_prev = rpm_filt;

    vTaskDelay(10);
    }
}

/** @brief Task which calculates the speed from a commanded torque
 * 
 *  @details This task calls the Controller class integrator to calculate a speed 
 *  command from a torque command. It runs once it detects a value in the torque_cmd queue
 *  so it runs at the rate of the outer control loop.
 */
void task_calcSetpoint(void* parameters) 
{
    while (true) 
    {
        float torque = torque_cmd.get();
        float omega = Controller_1.calculate_omega(torque);
        speed_cmd.put(omega);
        last_speed_cmd.put(omega);
    }
}

/** @brief Task which commands the speed using a state machine
 *  
 *  @details This task uses a state machine to command the speed of the motor. The motor driver has an 
 *  internal control loop for acceleration but not for deceleration. It has a pin that applies an on/off 
 *  BRAKE and a pin to control the direction. The state machine uses the commanded speed and actual speed 
 *  to switch between a controller-driven state, a deceleration state (which uses the brake pin), and 
 *  zero crossing states in each direction which switch the direction pin polarity in a deadband of 20rpm. 
 *  This state machine runs at a fixed rate of 100 Hz.
 */
void task_speedControl(void* parameters)
{
    // 0 = idle / stable, 1 = accel, 2 = decel, 3 = hi to lo crossing, 4 = lo to hi crossing
    uint8_t speed_state = 0; 

    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t last_time_us = micros();
    uint32_t now_us = 0;

    edge_count.put(10);

    float speed_command = 0.0f;      // initialize internal speed command variable to zero
    float speed_real = 0.0f;         // initialize internal actual speed variable to zero
    bool direction = LOW;           // initialize direction to positive (LO = + in this convention)
    Peripheral.set_dir(direction);  // set initial direction to positive
    float deadband = 20.0f;                // define deadband for state transitions (tuning parameter)
    float cmd_deadband = 40.0f;     // don't listen to commands less than +\- 20rpm to avoid chatter at zero (tuning parameter)
    float dt = 0.0f;
    float u = 0.0f;
    speed_cmd.put(0.0f); // initialize speed command queue to zero

    while (true) 
    {        

        now_us = micros();
        dt = (now_us - last_time_us) * 1.0e-6f;
        last_time_us = now_us;
        speed_real = speed_actual.get();  // every time the while loop runs it reads the actual speed
        direction = Peripheral.get_dir(); // read direction pin 
        speed_command = speed_cmd.get();
        
        u = Controller_1.PD(speed_command, speed_real, dt);

        if (fabsf(speed_real) < cmd_deadband)
        {
            speed_real = 0;
        }

        // controller-driven state
        if (speed_state == 0) // bury accel in state 0
        {
            Peripheral.unbrake();
            Peripheral.cmd_speed_SPI(u);
            // logic for + > ++, - > --, or - > +
            if (speed_command > speed_real && fabsf(speed_command) > cmd_deadband) 
            {
                if ((sign(speed_command) == sign(speed_real) || sign(speed_real) == 0) && (speed_command - speed_real > deadband)) 
                {
                    // positive or zero to larger positive
                    if (direction == LOW) 
                    {
                        // Command duty
                        Serial.println("accel");

                    }
                    // negative to smaller negative
                    else 
                    {
                        // decel
                        speed_state = 1; 
                        Serial.println("0 -> 1 negative");
                    }
                }
                // negative to positive
                else
                {
                    if (direction == HIGH && sign(speed_command) == 1)
                    {
                        Peripheral.cmd_speed_SPI(0);
                        Peripheral.brake();
                        // decel
                        speed_state = 1; 
                        Serial.println("0 -> 1 zerocross");
                    }
                }
            }

            // logic for ++ > +, -- > -, or + > -
            else if (speed_command < speed_real && fabsf(speed_command) > cmd_deadband) 
            {
                if ((sign(speed_command) == sign(speed_real) || sign(speed_real) == 0) && (speed_real - speed_command > deadband))
                {
                    // positive to smaller positive
                    if (direction == LOW)
                    {
                        // Command zero and brake the motor
                        // Peripheral.drv_write(0x00, 0x2122);
                        Peripheral.cmd_speed_SPI(0);
                        Peripheral.brake();

                        // decel
                        speed_state = 1; 
                        Serial.println("0 -> 1 positive");
                    }

                    // negative or zero to larger negative
                    else 
                    {                        
                        // Command speed
                        Peripheral.cmd_speed_SPI(u);
                        // Serial.println("accel");
                    
                    }
                }

                 // positive to negative
                else
                {
                    if (direction == LOW && sign(speed_command) == -1)
                    {
                        // decel
                        speed_state = 1;
                        Serial.println("0 -> 1 zerocross");
                    }
                }
            }
            else if (fabsf(speed_command) < cmd_deadband && fabsf(speed_real) > 100.0f) // different condition to avoid chatter between decel and idle at zero
            {
                speed_state = 1;
                Serial.println("0 -> 1 zerospd");
            }
        }
    
        // deceleration state
        else if (speed_state == 1) 
        {
            Peripheral.cmd_speed_SPI(0);
            Peripheral.brake();
            // no direction change
            if (sign(speed_command) == sign(speed_real) || sign(speed_command) == 0)
            { 
                // state transition from decel back to idle, deadband 20rpm
                if (fabsf(speed_command) > 100.0f)
                {
                    if (fabsf(speed_real-speed_command) <= deadband)
                    {
                        // return to stable state
                        speed_state = 0;
                        Serial.println("1 -> 0 non-deadband");
                    }
                }
                else // introduce a 40rpm deadband to prevent state lockup
                     // if speed command is very small, just command zero and brake
                {
                    if (fabsf(speed_real) <= 100.0f)
                    {
                        speed_state = 0;
                        Serial.println("1 -> 0 deadband");
                    }   
                }
            }

            // direction change handling
            if (sign(speed_command) != sign(speed_real) && sign(speed_command) != 0)
            {
                if (fabsf(speed_real) < 100.0f)
                {
                    if (speed_command > 0.0f)
                    {
                        speed_state = 2;
                        Serial.println("1->2 command positive");
                    }
                    else
                    {
                        speed_state = 3;
                        Serial.println("1->3 command negative");
                    }
                }
            }
        }


        // HI to LO zero crossing state
        // negative to positive speed
        // then state transition to accel
        else if (speed_state == 2) 
        {
            // change direction
            direction = LOW;
            Peripheral.set_dir(direction);

            // accel
            speed_state = 0;
            Serial.println("2->0");
        }

        // LO to HI zero crossing state
        // positive to negative speed
        // then state transition to accel
        else if (speed_state == 3) 
        {
            // change direction
            direction = HIGH;
            Peripheral.set_dir(direction);

            // accel
            speed_state = 0;
            Serial.println("3->0");
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));

    }
}

/** @brief The Arduino setup function which runs once at setup. 
 * 
 *  @details This function sets up the serial monitor, initializes the DRV8308 chip, 
 *  sets up the webserver, and initializes each task.
 */
void setup()
{
    // Start the serial port and delay to give time for the user to open it
    Serial.begin(115200);
    Wire.end();
    Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.begin((uint8_t)RW_I2C_ADDR);   // SLAVE: respond to this 7-bit address
    Wire.setClock(I2C_FREQ);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
    delay(6000);


    // Initialize the motor driver with default gains and settings as defined in Driver.cpp
    Peripheral.begin();
    Serial.println("DRV initialized");

    // Set up the webserver
    setup_wifi();

    // Set up the hardware counter
    // pcnt_init_falling_only((gpio_num_t)25);

    // Initialize shares so .get() does not block forever on first use
    speed_actual.put(0.0f);
    last_speed_cmd.put(0.0f);
    rw_fault_flags.put(0);

    // Task which runs the web server, handling live plotting and speed/torque/gain commanding
    // This task runs 10ms
    xTaskCreate (task_webserver, "Web Server", 8192, NULL, 1, NULL);

    // Task which calculates the actual speed of the motor based on an ISR
    // This task runs every time a rising edge is detected on FGOUT
    xTaskCreate(task_readActual, "Calculate RPM", 4096, NULL, 5, NULL);

    // Task which uses an integrator to calculate the speed from a commanded torque
    // This task runs every time a value is placed into torque_cmd
    // In the future this is how we will set our control loop frequency
    xTaskCreate(task_calcSetpoint, "Calculate Setpoint", 4096, NULL, 3, NULL);

    // Task which uses a state machine to command the motor speed
    // If in idle state, this task will not run until a value is placed into speed_cmd
    // This task will run every 10ms until back in idle state
    xTaskCreate(task_speedControl, "Speed Control", 4096, NULL, 4, NULL);

    xTaskCreate(task_gainWatchdog, "Gain Updates", 2048, NULL, 2, NULL);
}

/** @brief The Arduino loop function which we do not use here */
void loop()
{
    Serial.println(Peripheral.drv_read(0x2A));
    vTaskDelay(1000);
}
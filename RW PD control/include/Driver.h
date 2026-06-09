/** @file Driver.h
 *  This file contains a class written for the Texas Instruments DRV8308 motor driver chip.
 *  This class sets pins on the ESP32 which interact with respective pins on the driver,
 *  initializes the driver with default settings and gains, 
 *  and defines methods to control, command, and read the driver. 
 * 
 *  This code was written mostly by Neil Bedagkar and Bricen Rigby, with help from 
 *  Claude Sonnet 4.5 to write the ISR handling code.
*/

#pragma once

#include <Arduino.h>

class Driver 
{
    protected:

        uint16_t data16;            // initialize variable to store addresses
        uint8_t msb;                // initialize variable to read values off MISO
        uint8_t lsb;                // initialize variable to read values off MISO
        uint8_t PIN_SCLK;           // initialize pin for serial clock
        uint8_t PIN_MISO;           // initialize pin for MISO line
        uint8_t PIN_MOSI;           // initialize pin for MOSI line
        uint8_t PIN_SCS;            // initialize pin for chip select line
        uint8_t PIN_EN;             // initialize pin for ENABLE
        uint8_t PIN_FGOUT;          // initialize pin for FGOUT
        uint8_t PIN_CLKIN;          // initialize pin for CLKIN
        uint8_t PIN_FAULTn;         // initialize pin for FAULTn
        uint8_t PIN_LOCKn;          // initialize pin for LOCKn
        uint8_t PIN_RESET;          // initialize pin for RESET
        uint8_t PIN_BRAKE;          // initialize pin for BRAKE
        uint8_t PIN_DIR;            // initialize pin for DIR
        uint16_t FILK1;             // initialize FILK1 gain
        uint16_t FILK2;             // initialize FILK2 gain
        uint16_t COMPK1;            // initialize COMPK1 gain
        uint16_t COMPK2;            // initialize COMPK2 gain
        float SPI_REG;

        // Hardware timer variables for CLKIN fractional frequency generation
        hw_timer_t* clkin_timer;
        volatile double clkin_half_period_us;  // Half-period in microseconds
        volatile double clkin_accumulator;     // Accumulator for precise timing

    public:
        // Friend function for timer ISR (needs access to protected members)
        friend void timer_isr_handler();

        /** Non-inline functions are commented in Driver.cpp */
        Driver(void);
        Driver(uint8_t FILK1_, uint8_t FILK2_, uint8_t COMPK1_, uint8_t COMPK2_);


        void begin(void);



        /** @brief A function which sets the chip select pin to HIGH
         *  
         *  @details The DRV8308 SPI communication works on CS HIGH. Calling this 
         *  function begins a SPI transaction.
         */
        void scs_begin(void)
        {
            digitalWrite(PIN_SCS, HIGH);
        }
        


        /** @brief A function which sets the chip select pin to LOW
         *  
         *  @details The DRV8308 SPI communication works on CS HIGH. Calling this 
         *  function ends a SPI transaction.
         */
        void scs_end(void)
        {
            digitalWrite(PIN_SCS, LOW);
        }



        /** @brief A function which sets the enable pin to HIGH
         *  
         *  @details This initializes the DRV8308. It does not run on EN LOW.
        */
        void enable(void)
        {
            digitalWrite(PIN_EN, HIGH);
        }



        /** @brief A function which sets the enable pin to LOW 
         * 
         *  @details Disabling the DRV8308 also sets all of its internal registers to zero. This 
         *  occurs whenever the entire system is powered off, meaning that they have to be 
         *  reprogrammed whenever the system is turned back on (which is why the begin() method
         *  is called in the setup of main.cpp).
        */
        void disable(void)
        {
            digitalWrite(PIN_EN, LOW);
        }



        /** @brief A function which sets the brake pin to HIGH
         * 
         *  @details The DRV8308 does not have a deceleration control loop, meaning that if a 
         *  commanded speed is lower than the actual speed, if not commanded to brake, the 
         *  motor will coast until it reaches the lower speed. The brake and unbrake commands 
         *  are used in the state machine to prevent coasting.
         */
        void brake(void)
        {
            digitalWrite(PIN_BRAKE, HIGH);
        }



        /** @brief A function which sets the brake pin to LOW
         * 
         *  @details This function is called whenever the DRV is decelerating and reaches
         *  the commanded deadband. While within brake, the DRV will not read any commanded 
         *  PFM so it is necessary to unbrake before commanding a frequency.
         */
        void unbrake(void)
        {
            digitalWrite(PIN_BRAKE, LOW);
        }

    

        /** @brief A function which reads the direction
         * 
         *  @details This function reads the current polarity of the direction pin, which 
         *  tells us whether the motor is spinning in the positive or negative direction. 
         *  The convention defined for this project is 
         *  polarity LO = positive direction and 
         *  polarity HI = negative direction.
         * 
         *  @return The state of the direction pin. Logic for this is handled in the 
         *  readActual task.
         */
        bool get_dir(void)
        {
            return digitalRead(PIN_DIR);
        }



        /** @brief A function which sets the direction
         * 
         *  @details This function is used in the state machine during zero crossings to 
         *  reverse the direction of the motor by changing the polarity of the direction 
         *  pin to the desired. In the future, it may be desired to automate this further 
         *  by simply commanding the opposite polarity as opposed to a desired polarity.
         * 
         *  @param direction The desired direction polarity of the pin.
         */
        void set_dir(bool direction)
        {
            digitalWrite(PIN_DIR, direction);
        }

        void drv_write(uint8_t spdmode, uint16_t message);
        uint16_t drv_read(uint8_t addr7);

        void cmd_speed_PWM(float SPEED_CMD);
        void cmd_speed_SPI(float SPEED_CMD);
        // void debug_clkin_frequency();
        // double measure_clkin_frequency(uint16_t sample_time_ms = 1000);
        
        // PCNT functions
        void pcnt_begin_edge_isr(uint16_t threshold_edges);
        void pcnt_set_threshold(uint16_t threshold_edges);
        static void IRAM_ATTR pcnt_isr(void* arg);
        
        // Timer-based CLKIN PWM generation
        void clkin_timer_begin(void);

};


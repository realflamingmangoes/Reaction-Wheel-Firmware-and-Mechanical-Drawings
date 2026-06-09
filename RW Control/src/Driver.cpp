/** @file Driver.cpp
 *  This file contains a class written for the Texas Instruments DRV8308 motor driver chip.
 *  This class sets pins on the ESP32 which interact with respective pins on the driver,
 *  initializes the driver with default settings and gains, 
 *  and defines methods to control, command, and read the driver. 
 * 
 *  This code was written mostly by Neil Bedagkar and Bricen Rigby, with help from 
 *  Claude Sonnet 4.5 to write the ISR handling code.
*/

#include <Arduino.h>
#include <SPI.h>
#include "Driver.h"
#include "Shares.h"
#include <PrintStream.h>
#include "driver/pcnt.h"
#include "esp_err.h"

// Create a SPI class object
SPIClass vspi(VSPI);

// Global instance pointer for timer ISR callback
static Driver* g_driver_instance = nullptr;

// Timer ISR for CLKIN fractional frequency generation (friend function)
void IRAM_ATTR timer_isr_handler()
{
    if (g_driver_instance && g_driver_instance->clkin_half_period_us > 0.0) {
        g_driver_instance->clkin_accumulator += 10.0;  // Add 10 microsecond tick
        
        if (g_driver_instance->clkin_accumulator >= g_driver_instance->clkin_half_period_us) {
            g_driver_instance->clkin_accumulator -= g_driver_instance->clkin_half_period_us;
            digitalWrite(g_driver_instance->PIN_CLKIN, !digitalRead(g_driver_instance->PIN_CLKIN));
        }
    }
}

// ======== PCNT settings (legacy PCNT driver) ========
static constexpr pcnt_unit_t    PCNT_UNIT_USED = PCNT_UNIT_0;
static constexpr pcnt_channel_t PCNT_CH_USED   = PCNT_CHANNEL_0;

// If you count both edges of the FGOUT square wave, 2 edges = 1 cycle.
static constexpr float EDGES_PER_CYCLE = 2.0f; // set 1.0f if you count only rising OR only falling

static const int CH        = 0;   // LEDC channel
static const int RES_BITS  = 8;   // resolution (8-bit -> duty 0..255)

/** @brief Constructor for the Driver class
 * 
 *  @details This function initializes the DRV8308, setting each of the pins for the command and 
 *  default gains tuned for mediocre performance at all speeds 
 *  and during both idle and transient states.
 */
Driver::Driver(void)
{
    data16 = 0;
    msb = 0;
    lsb = 0;
    PIN_SCLK = 18;
    PIN_MISO = 19;
    PIN_MOSI = 23;
    PIN_SCS = 5;
    PIN_EN = 13;
    PIN_CLKIN = 14;
    PIN_FGOUT = 25;
    PIN_FAULTn = 26;
    PIN_LOCKn = 27;
    PIN_RESET = 15;
    PIN_BRAKE = 12;
    PIN_DIR = 16;
    FILK1 = 1000;
    FILK2 = 500;
    COMPK1 = 1000;
    COMPK2 = 1000;
    SPI_REG = 0;
}

/** @brief Constructor for the Driver class
 * 
 *  @details This function initializes the DRV8308, setting each of the pins for the command 
 *  setting custom gain coefficients. An accompanying MATLAB script is included to calculate 
 *  these gain coefficients using equations detailed in the DRV8308 datasheet.
 * 
 *  @param FILK1_ setting the coefficient for the filter pole
 *  @param FILK2_ setting the coefficient for the filter zero
 *  @param COMPK1_ setting the coefficient for the compensator pole
 *  @param COMPK2_ setting the coefficient for the compensator zero
 */
Driver::Driver(uint8_t FILK1_, uint8_t FILK2_, uint8_t COMPK1_, uint8_t COMPK2_)
{
    data16 = 0;
    msb = 0;
    lsb = 0;
    PIN_SCLK = 18;
    PIN_MISO = 19;
    PIN_MOSI = 23;
    PIN_SCS = 5;
    PIN_EN = 13;
    PIN_CLKIN = 14;
    PIN_FGOUT = 25;
    PIN_FAULTn = 26;
    PIN_LOCKn = 27;
    PIN_RESET = 15;
    PIN_BRAKE = 12;
    PIN_DIR = 16;
    FILK1 = FILK1_;
    FILK2 = FILK2_;
    COMPK1 = COMPK1_;
    COMPK2 = COMPK2_;
    SPI_REG = 0;
}

/** @brief A function which initializes the DRV8308
 * 
 *  @details This function sets each pin as an input or output, 
 *  attaches an interrupt to FGOUT so it can read square wave rising edges,
 *  enables the DRV8308, 
 *  unbrakes it from any previous operation,
 *  initializes the direction to be forward,
 *  begins SPI communication,
 *  writes initial gains to the DRV8308 chip,
 *  and sets up the CLKIN pin to output a square wave to output 50% duty cycle.
 */
void Driver::begin() 
{
    // Pin input/output definitions
    pinMode(PIN_SCS, OUTPUT);
    pinMode(PIN_EN, OUTPUT);
    pinMode(PIN_FGOUT, INPUT);
    pinMode(PIN_FAULTn, INPUT);
    pinMode(PIN_LOCKn, INPUT);
    pinMode(PIN_CLKIN, OUTPUT);         
    pinMode(PIN_RESET, OUTPUT);
    digitalWrite(PIN_RESET, LOW); // Keep RESET low for nominal operation
    pinMode(PIN_BRAKE, OUTPUT);   
    pinMode(PIN_DIR, OUTPUT);

    // Set up PCNT to interrupt every 100 edges on FGOUT
    pcnt_begin_edge_isr(10);

    // Initialize the driver
    enable();  // Enable the driver
    unbrake(); // Release BRAKE
    set_dir(LOW); // Set direction forward

    // SPI setup
    // PIN_SCS is set to -1 because we control that manually using scs_begin() and scs_end()
    // SPI defaults to ACTIVE LOW but the DRV8308 is ACTIVE HIGH so we control it manually
    vspi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);  // VSPI pins

    // Initial register programming (set SPDMODE to 00) (for clkin mode set 0x2000) & autogain setpt to 12hz
    // set BRKMOD to high (set SYNRECT to high for PID)
    drv_write(0x00, 0x2102);
    Serial.print("0x00 expect 0x2102, got 0x"); Serial.println(drv_read(0x00), HEX);


    // set MOD120 to 3970 as per DRV8308EVM users guide
    drv_write(0x03, 0b00000111110000010);
    Serial.print("0x03 expect 0xF82, got 0x"); Serial.println(drv_read(0x03), HEX);

    // set AUTOGAIN to 1
    drv_write(0x04, 0x0200);
    Serial.print("0x04 expect 0x0200, got 0x"); Serial.println(drv_read(0x04), HEX);

    // set SPDGAIN to 2048 and intclk to 000
    drv_write(0x05, 0x0800);
    Serial.print("0x05 expect 0x0800, got 0x"); Serial.println(drv_read(0x05), HEX);

    // set FILK1
    drv_write(0x06, FILK1);
    Serial.print("0x06 expect 0x007F, got 0x"); Serial.println(drv_read(0x06), HEX);

    // set FILK2
    drv_write(0x07, FILK2);
    Serial.print("0x07 expect 0x01FB, got 0x"); Serial.println(drv_read(0x07), HEX);

    // set COMPK1
    drv_write(0x08, COMPK1);
    Serial.print("0x08 expect 0x007F, got 0x"); Serial.println(drv_read(0x08), HEX);

    // set COMPK2
    // AUTOADV is here, setting to zero for now
    drv_write(0x09, COMPK2);
    Serial.print("0x09 expect 0x22A3, got 0x"); Serial.println(drv_read(0x09), HEX);

    // set LOOPGAIN to 511
    drv_write(0x0A, 511);
    Serial.print("0x0A expect 0x02BC, got 0x"); Serial.println(drv_read(0x0A), HEX);

    // set SPEED to 4095 so its just a toggle
    drv_write(0x0B, 0x0FFF);
    Serial.print("0x0B expect 0x0FFF, got 0x"); Serial.println(drv_read(0x0B), HEX);
    // Setup CLKIN for square wave output using hardware timer ISR
    // The internal control loop in the DRV8308 matches the frequency input on CLKIN 
    // to the motor electrical frequency output on FGOUT
    clkin_timer_begin();  // Initialize timer-based fractional frequency generation
}

/** @brief A function which writes a register on the DRV8308
 * 
 *  @details This function takes an 8-bit address and a 16-bit message, 
 *  and uses SPI to write a register on the DRV8308. This code was written
 *  with the help of ChatGPT 5.1.
 * 
 *  @param addr7 the 7-bit address register. The first bit is always zero 
 *  to signify a WRITE operation to the DRV8308 chip.
 * 
 *  @param message the 16-bit number that is being written to the DRV8308.
 *  Registers are defined on the DRV8308 datasheet.
 */
void Driver::drv_write(uint8_t addr7, uint16_t message) 
{
    vspi.beginTransaction(SPISettings(10000, MSBFIRST, SPI_MODE0)); // 1 MHz, Mode 0
    scs_begin();
    delayMicroseconds(1); // Setup time for SCS
    vspi.transfer((0u << 7) | (addr7 & 0x7F));
    vspi.transfer((uint8_t)(message >> 8));
    vspi.transfer((uint8_t)(message & 0xFF));
    delayMicroseconds(1); // Hold time for data
    scs_end();
    vspi.endTransaction();
    delayMicroseconds(5); // Recovery time between transactions
}

/** @brief A function which reads a register on the DRV8308
 *  
 *  @details This function takes an 8-bit address and uses SPI to read 
 *  that register on the DRV8308. It returns the 16-bit value of that 
 *  register. This was developed primarily to debug SPI communication 
 *  and was kept because of its usefulness, and just in case the SPI
 *  demons decide to haunt us again. This code was written with the
 *  help of ChatGPT 5.1.
 * 
 *  @param addr7 the 7-bit address register. The first bit is always one
 *  to signify a READ operation to the DRV8308 chip.
 * 
 *  @return the 16 bit value of the register on the DRV8308.
 */
uint16_t Driver::drv_read(uint8_t addr7)  
{
    vspi.beginTransaction(SPISettings(10000, MSBFIRST, SPI_MODE0)); // Same mode as write
    scs_begin();
    delayMicroseconds(1); // Setup time for SCS
    vspi.transfer((1u << 7) | (addr7 & 0x7F));
    msb = vspi.transfer(0x00);
    lsb = vspi.transfer(0x00);
    delayMicroseconds(1); // Hold time for data
    scs_end();
    vspi.endTransaction();
    delayMicroseconds(5); // Recovery time
    return (uint16_t(msb) << 8) | lsb;
}

/** @brief A function which commands a square wave to the CLKIN pin
 * 
 *  @details This function writes a square wave of 50% duty cycle to the CLKIN pin
 *  at the desired electrical frequency of the motor. From an equation on the 
 *  DRV8308 datasheet, the electrical frequency is equal to RPM / 15.
 *  Uses a hardware timer with fractional frequency support.
 * 
 *  @param SPEED_CMD The desired rpm speed of the motor.
 */
void Driver::cmd_speed_PWM(float SPEED_CMD)
{
    double freq = SPEED_CMD / 15.0;  // Electrical frequency in Hz
    
    // Calculate half-period in microseconds (with full double precision)
    if (freq > 0.0) {
        clkin_half_period_us = 500000.0 / freq;
    } else {
        clkin_half_period_us = 0.0;  // Stop output if frequency is 0 or negative
    }
}

/** @brief A function which commands PWM directly to the MOSFETS
 * 
 *  @details This function commands a desired duty cycle to the DRV8308 by writing to the SPEED register. 
 * 
 *  @param SPEED_CMD The desired rpm speed of the motor.
 */
void Driver::cmd_speed_SPI(float SPEED_CMD)
{
    if (SPEED_CMD == -1)
    {
        SPI_REG = 4095;
    }
    else
    {
        SPI_REG = (0.4279370635* SPEED_CMD) + 63.6341234403;
    }
    
    drv_write(0x0B, (uint16_t)SPI_REG);
}


// ============ Interrupt-based edge detection ============ //
void Driver::pcnt_begin_edge_isr(uint16_t threshold_edges)
{
    // Configure PCNT to count edges on FGOUT (PIN_FGOUT)
    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = PIN_FGOUT;
    cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
    cfg.unit           = PCNT_UNIT_USED;
    cfg.channel        = PCNT_CH_USED;

    // Count BOTH edges for better resolution
    cfg.pos_mode = PCNT_COUNT_INC;   // rising
    cfg.neg_mode = PCNT_COUNT_INC;   // falling
    cfg.lctrl_mode = PCNT_MODE_KEEP;
    cfg.hctrl_mode = PCNT_MODE_KEEP;

    cfg.counter_h_lim = 32767;
    cfg.counter_l_lim = 0;

    ESP_ERROR_CHECK(pcnt_unit_config(&cfg));

    // Optional filter (APB cycles). Tune if needed.
    ESP_ERROR_CHECK(pcnt_set_filter_value(PCNT_UNIT_USED, 80));
    ESP_ERROR_CHECK(pcnt_filter_enable(PCNT_UNIT_USED));

    // Configure threshold event
    ESP_ERROR_CHECK(pcnt_set_event_value(PCNT_UNIT_USED, PCNT_EVT_THRES_1, (int16_t)threshold_edges));
    ESP_ERROR_CHECK(pcnt_event_enable(PCNT_UNIT_USED, PCNT_EVT_THRES_1));

    // Enable interrupts for this PCNT unit
    ESP_ERROR_CHECK(pcnt_intr_enable(PCNT_UNIT_USED));

    // Install ISR service once
    static bool isr_service_installed = false;
    if (!isr_service_installed) {
        ESP_ERROR_CHECK(pcnt_isr_service_install(0));
        isr_service_installed = true;
    }

    // Register ISR for this unit; pass `this` in case you want it later
    ESP_ERROR_CHECK(pcnt_isr_handler_add(PCNT_UNIT_USED, Driver::pcnt_isr, this));

    // Start counter
    ESP_ERROR_CHECK(pcnt_counter_pause(PCNT_UNIT_USED));
    ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT_USED));
    ESP_ERROR_CHECK(pcnt_counter_resume(PCNT_UNIT_USED));
}

/** @brief Change PCNT threshold safely from task context
 *
 *  @details Call this from a task (not from ISR). It pauses counting and
 *  disables PCNT interrupts while updating the threshold event value,
 *  optionally clears the counter for deterministic behavior, then
 *  resumes counting and re-enables interrupts.
 */
void Driver::pcnt_set_threshold(uint16_t threshold_edges)
{
    // Disable PCNT interrupts and pause counter while reconfiguring
    ESP_ERROR_CHECK(pcnt_intr_disable(PCNT_UNIT_USED));
    ESP_ERROR_CHECK(pcnt_counter_pause(PCNT_UNIT_USED));

    // Update threshold event value
    ESP_ERROR_CHECK(pcnt_set_event_value(PCNT_UNIT_USED, PCNT_EVT_THRES_1, (int16_t)threshold_edges));

    // Clear counter so the new threshold starts fresh (optional but deterministic)
    ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT_USED));

    // Resume counting and re-enable interrupts
    ESP_ERROR_CHECK(pcnt_counter_resume(PCNT_UNIT_USED));
    ESP_ERROR_CHECK(pcnt_intr_enable(PCNT_UNIT_USED));
}

/** @brief Initialize hardware timer for CLKIN fractional frequency generation
 * 
 *  @details Sets up Timer 1 to generate precise fractional frequency square waves
 *  on the CLKIN pin using an accumulator-based approach. This allows frequencies
 *  like 66.67 Hz without rounding.
 */
void Driver::clkin_timer_begin(void)
{
    // Set global instance pointer for ISR
    g_driver_instance = this;
    
    // Initialize timer variables
    clkin_half_period_us = 0.0;     // Start with no output
    clkin_accumulator = 0.0;
    
    // Configure and attach Timer 1
    // Timer 1, 80x prescaler: 80MHz / 80 = 1MHz = 1 microsecond per tick
    clkin_timer = timerBegin(1, 80, true);
    timerAttachInterrupt(clkin_timer, timer_isr_handler, true);
    timerAlarmWrite(clkin_timer, 10, true);  // ISR fires every 10 microseconds
    timerAlarmEnable(clkin_timer);
}

void IRAM_ATTR Driver::pcnt_isr(void* arg)
{
    (void)arg;

    // Reading event status clears the event latch for this unit.
    uint32_t status = 0;
    pcnt_get_event_status(PCNT_UNIT_USED, &status);

    if (status & PCNT_EVT_THRES_1) {
        // Timestamp immediately
        uint32_t t_us = micros();

        // Push timestamp to your queue (must be ISR-safe)
        edge_time.put(t_us);
    }
}
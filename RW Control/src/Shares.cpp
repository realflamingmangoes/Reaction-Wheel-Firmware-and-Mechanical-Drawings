#include "Shares.h"

// A share which holds the last torque command for Controller integration
Queue<float> torque_cmd (2, "Torque Command");

// A queue which holds speed command values from the webserver and passes them to the speedControl state machine
Share<float> speed_cmd ("Speed Command"); 

// A share which holds the last speed command for Controller integration
Share<float> last_speed_cmd ("Last Speed Command"); 

// A share which populates using an ISR and holds the current speed of the motor
Share<float> speed_actual ("Speed Actual");

// Queues which uses an ISR to trigger the readActual task and calculate the motor speed by reading the square wave 
// frequency on the FGOUT pin
Queue<uint32_t> edge_time (4, "Rising Edge Timestamp");
Queue<float> edge_count (2, "Edge Count");

// Queues which hold live-tuned gains from the webserver interface
Queue<float> kp (2, "Kp");
Queue<float> kd (2, "Kd");

// A share which stores the fault state of the reaction wheel
Share<uint32_t> rw_fault_flags("RW Fault Flags");
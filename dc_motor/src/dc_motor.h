#ifndef _DC_MOTOR_H_
#define _DC_MOTOR_H_

// Include main board configuration file
#include "board_cfg.h"
#include <stdint.h>

// PWM configuration
// -----------------

// PWM Frequency, in Hz
#define DC_MOTOR_FREQ 20000

// Equivalent period for register configuration
#define DC_MOTOR_PER ((FCY/DC_MOTOR_FREQ)-1)

// Maximum speed : remmember that it is doubled because
// TMR<14:0> is compared to DC<15:1>
#define DC_MOTOR_MAX (2*(FCY/DC_MOTOR_FREQ)-1) 

// Prototypes
// ----------

void dc_motor_init(void);
void dc_motor_set_speed(uint8_t pwm_no, int16_t val);

#endif // !_DC_MOTOR_H_

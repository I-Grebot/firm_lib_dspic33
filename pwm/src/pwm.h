#ifndef _PWM_H_
#define _PWM_H_
// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------
#include "board_cfg.h"

// -----------------------------------------------------------------------------
// DEFINES
// -----------------------------------------------------------------------------
#define FREE_RUNNING_MODE                   0b00
#define SINGLE_EVENT_MODE                   0b01
#define COUNTING_MODE                       0b10
#define COUNTING_MODE_WITH_INTERRUPT        0b11
#define PWM1                                1
#define PWM2                                2

// -----------------------------------------------------------------------------
// FREQUENCIES
// -----------------------------------------------------------------------------
#define FPWM                                20000       // pwm frequency = 20kHz
#define PWM_POSTSCALER                          0       // postscaler 1:1
#define PWM_PRESCALER                           0       // prescaler 1:1
#define PWM_PERIOD  (FCY/FPWM)-1            // Period = 1999

// -----------------------------------------------------------------------------
// MACROS
// -----------------------------------------------------------------------------
#define pwm1_halts_in_CPU_idle          do{P1TCONbits.PTSIDL = 1;}while(0)
#define pwm2_halts_in_CPU_idle          do{P2TCONbits.PTSIDL = 1;}while(0)
#define pwm1_runs_in_CPU_idle           do{P1TCONbits.PTSIDL = 0;}while(0)
#define pwm2_runs_in_CPU_idle           do{P2TCONbits.PTSIDL = 0;}while(0)
#define pwm1_pin_pairs_independent      do{PWM1CON1bits.PMOD1 = 1;}while(0)
#define pwm1_pin_pairs_complementary    do{PWM1CON1bits.PMOD1 = 0;}while(0)
#define pwm2_pin_pairs_independent      do{PWM1CON1bits.PMOD2 = 1;}while(0)
#define pwm2_pin_pairs_complementary    do{PWM1CON1bits.PMOD2 = 0;}while(0)

/** 
 * Initialisation of pwm, variables
 *
 * \param postcaler : a uint8_t contaning the postscaler of the pwm frequency
 *
 * \param prescaler : a uint8_t contaning the prescaler from FCY
 *
 * \param mode : a uint8_t containing the mode of functionement for pwm module
 */
void pwm_init(uint8_t postscaler, uint8_t prescaler, uint8_t mode);

/** Extract pwm duty cycle.
 *
 * \param data : a (uint8_t) containing the number of the pwm to be read.
 *               
 */
uint16_t pwm_get_duty(uint8_t number);


/** Set a pwm duty cycle
 *
 * \param data : a (uint8_t) containing the number
 *               of the pwm to set.
 * \param v    : the value
 */
void pwm_set_duty(uint8_t number, uint16_t val);

void pwm_set_period(uint16_t val);
uint16_t pwm_get_period(void);

#endif

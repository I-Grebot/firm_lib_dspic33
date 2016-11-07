/** \file pwm.c
 *  \brief Implementation for setting motor speed
 *
 */

#include <pwm.h>

/**
 * Initialisation of pwm, variables
 *
 * \param postcaler : a uint8_t contaning the postscaler of the pwm frequency
 *
 * \param prescaler : a uint8_t contaning the prescaler from FCY
 *
 * \param mode : a uint8_t containing the mode of functionement for pwm module
 */
void pwm_init(uint8_t postscaler, uint8_t prescaler, uint8_t mode)
{
    P1TCONbits.PTEN = 0;
    P1TCONbits.PTOPS = postscaler;      // set pwm postscaler
    P1TCONbits.PTCKPS = prescaler;      // set pwm prescaler
    P1TCONbits.PTMOD = mode;            // set pwm mode
    P1TPERbits.PTPER = 0;               // reset pwm period
    PWM1CON2bits.UDIS = 0;
    PWM1CON2bits.IUE = 1;               // immediate update
    PWM1CON1bits.PMOD1 = 1;             // pin pair 1 in independant mode
    PWM1CON1bits.PMOD2 = 1;             // pin pair 2 in independant mode
    PWM1CON1bits.PEN1L = 0;
    PWM1CON1bits.PEN1H = 1;
    PWM1CON1bits.PEN2L = 1;
    PWM1CON1bits.PEN2H = 0;
    P1TCONbits.PTEN = 1;                // pwm1 timer is on
}

/** Extract pwm duty cycle.
 *
 * \param number : a (uin16_t) containing the number of the pwm duty
 *                 cycle to be read.
 */
uint16_t pwm_get_duty(uint8_t number)
{
    uint16_t value;

    switch (number)
    {
        case 1:
            value = P1DC1;
            break;
        case 2:
        default:
            value = P1DC2;
            break;
    }

    return value;
}

/** Set a pwm duty cycle
 *
 * \param number : a (uint8_t) containing the number of the pwm duty
 *                 cycle to be set.
 * \param val    : the value
 */
void pwm_set_duty(uint8_t number, uint16_t val)
{
    switch (number)
    {
        case 1:
            P1DC1 = val;
            break;
        case 2:
        default:
            P1DC2=val;
            break;
    }
}

/** Get a pwm period
 *
  * \return value      : the period of the pwm module
 */
uint16_t pwm_get_period(void)
{
    uint16_t value;
    value = P1TPER;

    return value;
}

/** Set a pwm period
 *
 * \param v      : the value
 */
void pwm_set_period(uint16_t v)
{
    P1TCONbits.PTEN = 0;
    P1TPER = v;
    P1TCONbits.PTEN = 1;
}

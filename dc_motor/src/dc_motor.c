
#include "dc_motor.h"

void dc_motor_init(void) {

    // Set directions to outputs
#ifdef DC_MOTOR_USE_PWM1H1
    DC_MOTOR_PWM_PWM1H1_TRIS = 0;
    DC_MOTOR_DIR_PWM1H1_TRIS = 0;
    PWM1CON1bits.PEN1H  = 1;             // PWM1H1 pin is enabled
#endif

#ifdef DC_MOTOR_USE_PWM1H2
    DC_MOTOR_PWM_PWM1H2_TRIS = 0;
    DC_MOTOR_DIR_PWM1H2_TRIS = 0;
    PWM1CON1bits.PEN2H  = 1;             // PWM1H2 pin is enabled
#endif

#ifdef DC_MOTOR_USE_PWM1H3
    DC_MOTOR_PWM_PWM1H3_TRIS = 0;
    DC_MOTOR_DIR_PWM1H3_TRIS = 0;
    PWM1CON1bits.PEN3H  = 1;             // PWM1H3 pin is enabled
#endif

#ifdef DC_MOTOR_USE_PWM2H1
    DC_MOTOR_PWM_PWM2H1_TRIS = 0;
    DC_MOTOR_DIR_PWM2H1_TRIS = 0;
#endif

#if defined(DC_MOTOR_USE_PWM1H1) || defined(DC_MOTOR_USE_PWM1H2) || defined(DC_MOTOR_USE_PWM1H3)
    // Configure PWM1 module
    P1TCONbits.PTCKPS   = 0;             // 1:1 prescaler
    P1TCONbits.PTMOD    = 0;             // Free running mode
    P1TPERbits.PTPER    = DC_MOTOR_PER ; // Configure PWM frequency
    P1TCONbits.PTOPS    = 0;             // 1:1 postscale
    P1DC1               = 0;             // Init duty-cycle 1 to 0
    PWM1CON2bits.UDIS   = 0;             // Updates from PER and PDC are enabled

    P1OVDCONbits.POVD1H = 1;             // PWM I/O pin controlled by PWM generator
    P1TCONbits.PTEN     = 1;             // Enable PWM 1 module
#endif

#ifdef DC_MOTOR_USE_PWM2H1
    // Configure PWM2 module
    P2TCONbits.PTCKPS   = 0;             // 1:1 prescaler
    P2TCONbits.PTMOD    = 0;             // Free running mode
    P2TPERbits.PTPER    = DC_MOTOR_PER ; // Configure PWM frequency
    P2TCONbits.PTOPS    = 0;             // 1:1 postscale
    P2DC1               = 0;             // Init duty-cycle 1 to 0
    PWM2CON2bits.UDIS   = 0;             // Updates from PER and PDC are enabled

    P2OVDCONbits.POVD1H = 1;
    P2TCONbits.PTEN     = 1;
#endif
    
#ifdef DC_MOTOR_USE_PWM2H1
    PWM2CON1bits.PEN1H  = 1;             // PWM2H1 pin is enabled
#endif
}

void dc_motor_set_speed(uint8_t pwm_no, sint16_t val) {

    uint8_t dir;

    // Ensure pwm_no is on 2 bits
    pwm_no &= 0x3;

    // Check that value is correct
    if(val >= 0)
    {
        dir = 0;
    }else
    {
        if(val<-DC_MOTOR_MAX) val=-DC_MOTOR_MAX; // Clamp
        dir = 1;
        val=-val; // PWM value is positive
    }

    // Affect correct pwm value and direction
    switch(pwm_no) {

        #ifdef DC_MOTOR_USE_PWM1H1
        case 0: P1DC1 = val; DC_MOTOR_DIR_PWM1H1 = dir; break;
        #endif

        #ifdef DC_MOTOR_USE_PWM1H2
        case 1: P1DC2 = val; DC_MOTOR_DIR_PWM1H2 = dir; break;
        #endif

        #ifdef DC_MOTOR_USE_PWM1H3
        case 2: P1DC3 = val; DC_MOTOR_DIR_PWM1H3 = dir; break;
        #endif

        #ifdef DC_MOTOR_USE_PWM2H1
        case 3: P2DC1 = val; DC_MOTOR_DIR_PWM2H1 = dir; break;
        #endif

        default:break;
    }
}

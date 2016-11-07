//timer1.h contain functions related to Timer1's use
#ifndef _TIMER1_H_
#define _TIMER1_H_

// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------
#include <p33Fxxxx.h>
#include "board_cfg.h"

// -----------------------------------------------------------------------------
// MACROS
// -----------------------------------------------------------------------------
#define timer1_start do{T1CONbits.TON = 1;}while(0)
#define timer1_stop do{T1CONbits.TON = 0;}while(0)
#define timer1_reset do{TMR1 = 0;}while(0)
#define timer1_enable_interrupt do{IEC0bits.T1IE = 1;}while(0)
#define timer1_disable_interrupt do{IEC0bits.T1IE = 0;}while(0)
#define timer1_clear_flag do{IFS0bits.T1IF = 0;}while(0)

// -----------------------------------------------------------------------------
// PROTOTYPES
// -----------------------------------------------------------------------------
void timer1_init(uint16_t period);
void timer1_setup_interrupt(uint8_t priority);
 
#endif


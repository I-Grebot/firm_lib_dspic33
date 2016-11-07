//timer.c
#include "timer1.h"

 // Timer1 init
 void timer1_init(uint16_t period)
 {
     timer1_stop;
     T1CONbits.TGATE = 0;
     T1CONbits.TCS  =  0;               // Out = Fcy
     T1CONbits.TCKPS = 3;               // Out = Fcy/256
     timer1_reset;
     PR1 = period;                      
     timer1_setup_interrupt(1);         // lower priority
 }

 void timer1_setup_interrupt(uint8_t priority)
 {
     IPC0bits.T1IP = priority;
     timer1_clear_flag;
     timer1_enable_interrupt;
 }

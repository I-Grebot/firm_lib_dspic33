/* -----------------------------------------------------------------------------
 * I-Grebot Scheduler Library
 * -----------------------------------------------------------------------------
 * File        : scheduler.c
 * Language    : C
 * Author      : Paul M.
 * Creation    : 2013-01-13
 * -----------------------------------------------------------------------------
 * Description
 *   This is a basic scheduling implementation.
 * -----------------------------------------------------------------------------
 * Versionning informations
 * Repository: http://svn2.assembla.com/svn/paranoid_android/
 * -----------------------------------------------------------------------------
 * $Rev:: 1269                                                                 $
 * $LastChangedBy:: paul.m                                                     $
 * $LastChangedDate:: 2015-03-18 23:09:43 +0100 (mer., 18 mars 2015)           $
 * -----------------------------------------------------------------------------
 * Version     Comment                                   Author       Date
 * 1.0         Initial release                           Paul M.      2013-01-13
 * -----------------------------------------------------------------------------
 */

#include "board_cfg.h"

// Number of scheduler ticks since started
uint32_t tick_cnt;

// Software IRQ vector
extern uint8_t sw_irq;

// -----------------------------------------------------------------------------
// SCHEDULER
// -----------------------------------------------------------------------------

#define SCHEDULER_TIMER_PERIOD ((FCY/1280000)*SCHEDULER_PERIOD_50_US)
void scheduler_init(void) {

    // Init Timer 5 for schedulling,
    // Use timer as a regular 16 bits timer
    // It is assumed that Timer 4 is used as a standalone 16 bits timer
    T5CONbits.TON   = 0     ; // Disable Timer
    T5CONbits.TCS   = 0     ; // Internal cycle clock selected
    T5CONbits.TCKPS = 0b10  ; // Set prescaler = 1:64 ; TIM5 Freq = FCY/64
    TMR5            = 0     ; // Clear Timer
    PR5             = SCHEDULER_TIMER_PERIOD ; // Select TIM5 Period

    // Enable Timer5 Interrupt
    IFS1bits.T5IF = 0;
    IEC1bits.T5IE = 1;
}

uint8_t scheduler_ready(void) {
    if(sw_irq & IRQ_SCHEDULER_READY) {
      sw_irq &= (~IRQ_SCHEDULER_READY);
      return 1;
    } else {
      return 0;
    }
}


void scheduler_start(void) {
    tick_cnt      = 0 ;
    T5CONbits.TON = 1 ;
}

void scheduler_stop(void) {
  T5CONbits.TON = 0 ;
}

uint16_t scheduler_task_ready(uint16_t _rate) {
  return !(tick_cnt % _rate);
}

void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {
  tick_cnt++;
  sw_irq |= IRQ_SCHEDULER_READY ;
  IFS1bits.T5IF = 0; // Clear interrupt
}

#ifndef _BOARD_H_
#define _BOARD_H_

// Hardware main include
#include <p33Fxxxx.h>

// Compiler related types
typedef unsigned char uint8_t ;
typedef signed   char sint8_t ;
typedef unsigned int  uint16_t;
typedef signed   int  sint16_t;
typedef unsigned long uint32_t;
typedef signed   long sint32_t;



// -----------------------------------------------------------------------------
// FREQUENCIES
// -----------------------------------------------------------------------------

// unused from here...
// Tune the Fast RC Oscillator to reach roughly 8MHz
// 23*0.375% = +8.625%
// FOSC = 7.37*1.08625 = 8.0056625 MHz
// => Error = 0.071%
//#define FRC_BASE_FREQ 7370000 // 7.37 MHz, Cf. datasheet
//#define FRC_TUNE 23
//
//// FOSC = (FIN/N1) * M / N2
//// N1 = 2, M = 32, N2 = 2
//#define PLL_RATIO (32/(2*2))

// ... to here

// Add-up all frequency tunnings
//64045300 // (FRC_BASE_FREQ*(1+0.00375*FRC_TUNE)*PLL_RATIO)
#define FOSC 80000000//79227500 // 7.37 * 43 / 4
#define FCY  (FOSC/2)

// -----------------------------------------------------------------------------
// HARDWARE
// -----------------------------------------------------------------------------

#define LED_TRIS     (TRISAbits.TRISA9)   // LED        mapped on RA9
#define LED          (LATAbits.LATA9)


// Define debug UART IOs
// ---------------------
//#define UART_RX_A_TRIS (TRISBbits.TRISB9)   // UART_RX_A    mapped on RB9/RP9
//#define UART_RX_A_PIN  9
//
//#define UART_TX_A_TRIS (TRISBbits.TRISB8)   // UART_TX_A    mapped on RB8/RP8
//#define UART_TX_A_RPN  (RPOR4bits.RP8R)
//#define UART_TX_PIN 0b00011
//
//#define DEBUG_UART_BAUDRATE 115200
//#define DEBUG_UART_BRG ((FCY/16/DEBUG_UART_BAUDRATE)-1)

// Define DC-motors constants
// --------------------
// Motor 1
#define DC_MOTOR_PWM_PWM1H1_TRIS  (TRISBbits.TRISB14)  // DC_PWM_5     mapped on RB14/PWM1H1
#define DC_MOTOR_DIR_PWM1H1_TRIS  (TRISBbits.TRISB15)  // DC_DIR_5     mapped on RB15/RP15
#define DC_MOTOR_DIR_PWM1H1       (LATBbits.LATB15)

// Motor 2
#define DC_MOTOR_PWM_PWM1H2_TRIS  (TRISBbits.TRISB12)  // DC_PWM_6     mapped on RB12/PWM1H2
#define DC_MOTOR_DIR_PWM1H2_TRIS  (TRISBbits.TRISB13)  // DC_DIR_6     mapped on RB13/RP13
#define DC_MOTOR_DIR_PWM1H2       (LATBbits.LATB13)

// Define which dc motor to use
#define DC_MOTOR_USE_PWM1H1 // Channel 0
#define DC_MOTOR_USE_PWM1H2 // Channel 1

// Define Encoders IOs
// -------------------

// Encoder 1
#define ENC_A_1_TRIS   (TRISCbits.TRISC0)   // ENC_A_1      mapped on RC0/RP16
#define ENC_A_1_PIN    16
#define ENC_A_1        (PORTCbits.RC0)

#define ENC_B_1_TRIS   (TRISCbits.TRISC1)   // ENC_B_1      mapped on RC1/RP17
#define ENC_B_1_PIN    17
#define ENC_B_1        (PORTCbits.RC1)

#define ENC_I_1_TRIS   (TRISCbits.TRISC2)   // ENC_I_1      mapped on RC2/RP18
#define ENC_I_1_PIN    18
#define ENC_I_1        (PORTCbits.RC2)

// Encoder 2
#define ENC_A_2_TRIS   (TRISCbits.TRISC3)   // ENC_A_2      mapped on RC3/RP19
#define ENC_A_2_PIN    19
#define ENC_A_2        (PORTCbits.RC3)

#define ENC_B_2_TRIS   (TRISCbits.TRISC4)   // ENC_B_2      mapped on RC4/RP20
#define ENC_B_2_PIN    20
#define ENC_B_2        (PORTCbits.RC4)

#define ENC_I_2_TRIS   (TRISCbits.TRISC5)   // ENC_I_2      mapped on RC5/RP21
#define ENC_I_2_PIN    21
#define ENC_I_2        (PORTCbits.RC5)


#endif

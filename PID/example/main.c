/* -----------------------------------------------------------------------------
 * pid library example file
 *
 * This example provide two independant motor controls, using dsPIC33F QEI peripherals
 *
 * Board used is IGrevator_V2, uC A
 *
 * -----------------------------------------------------------------------------
 * Reminder: Ziegler-Nichols method:
 *
 * Ki,Kd=0
 * Find Kp_lim were system oscillate and measure the oscillation period Tosc
 *
 * Then compute the PID gains as follow:
 *
 * Kp = 0.6 Kp_lim
 * Ki = 1/(0.5 Tosc)
 * Kd = 0.125 Tosc
 *
 * If gains are too low, sampling rate is too low!
 */

#include "board_cfg.h"
#include "pid.h"
#include <stdlib.h>

_FOSCSEL(FNOSC_FRCPLL)
_FOSC(FCKSM_CSECMD & IOL1WAY_OFF & OSCIOFNC_ON & POSCMD_NONE)
_FICD(JTAGEN_OFF)

_FWDT(FWDTEN_OFF);

// PID Processes
PID_process_t *pPID_1;
PID_process_t *pPID_2;
PID_process_t *sPID_1;
PID_process_t *sPID_2;

void osc_init(void)
{
    // Configure PLL prescaler, PLL postscaler, PLL divisor
    PLLFBD=38;                      // M=40
    CLKDIVbits.PLLPRE=0;            // N1=2
    CLKDIVbits.PLLPOST=0;           // N2=2
    RCONbits.SWDTEN = 0;            // Disable Watch Dog Timer
    OSCTUN = 23;                    // 7,37MHz x 8,625% = 8MHz

    // Clock switch to incorporate PLL
    __builtin_write_OSCCONH(0x01);  // Initiate Clock Switch to FRC with PLL (NOSC=0b001)
    __builtin_write_OSCCONL(0x01);  // Start clock switching

    while(OSCCONbits.COSC != 0b001);// Wait for Clock switch to occur
    while(OSCCONbits.LOCK!=1);      // Wait for PLL to lock
}

void io_init(void)
{
    // All I/Os are digital by default
    ADPCFG = 0xFFFF;
    AD1PCFGL = 0xFFFF;

    LED_TRIS=0;

    // UART 1 used for debug purpose
//    UART_RX_A_TRIS=1;
//    RPINR18bits.U1RXR=UART_RX_A_PIN;
//    UART_TX_A_TRIS=0;
//    UART_TX_A_RPN=UART_TX_PIN;  //RPO tied to UART1 Transmit
    
    DC_MOTOR_PWM_PWM1H1_TRIS=0;
    DC_MOTOR_DIR_PWM1H1_TRIS=0;
    DC_MOTOR_PWM_PWM1H2_TRIS=0;
    DC_MOTOR_DIR_PWM1H2_TRIS=0;

    ENC_A_1_TRIS=1;
    ENC_B_1_TRIS=1;
    ENC_I_1_TRIS=1;
    ENC_A_2_TRIS=1;
    ENC_B_2_TRIS=1;
    ENC_I_2_TRIS=1;
    RPINR14bits.QEA1R = ENC_A_1_PIN ;
    RPINR14bits.QEB1R = ENC_B_1_PIN;
    RPINR16bits.QEA2R = ENC_A_2_PIN;
    RPINR16bits.QEB2R = ENC_B_2_PIN;
}


void timer_init(void)
{
    // Timer 4 will call our PID processes
    T4CONbits.TON   = 0     ; // Disable Timer
    T4CONbits.TCS   = 0     ; // Internal cycle clock selected
    T4CONbits.T32   = 0     ; // 16 bits
    T4CONbits.TCKPS = 0b10  ; // Set prescaler = 1:64 ; TIM4 Freq = FCY/64
    TMR4            = 0     ; // Clear Timer
    PR4             = 50000 ; // Select TIM4 Period
    T4CONbits.TON   = 1     ; // Start Timer
    // Enable Timer 4 Interrupt
    IFS1bits.T4IF = 0;
    IEC1bits.T4IE = 1;
}

int main(void)
{
    osc_init();
    io_init();
    encoders_init();
    dc_motor_init();
	/*Memory alloc for the process pointers */
    /*Do not forget to set a heap size in the linker option*/
    pPID_1 = pid_init(0,0,1);		//position PID 1
	PID_Set_Coefficient(pPID_1->PID,3,0,3,0);
	PID_Set_limitation(pPID_1,2000,0);
	PID_Set_Ref_Position(PID_1,10000);
	
	pPID_2 = pid_init(1,1,1);		//position PID 2 with speed control sPID_1
	PID_Set_Coefficient(pPID_2->PID,3,1,3,200);
	PID_Set_limitation(pPID_2,0,0);
	PID_Set_Ref_Position(PID_2,500000);
	
	sPID_1 = pid_init(1,1,1);		//speed PID	
	PID_Set_Coefficient(sPID_1->PID,50,20,20,0);
	PID_Set_limitation(sPID_1,4000,0);
	
	sPID_2 = pid_init(2,2,1);		//speed PID	- QEI3 doesn't exist only for example
	PID_Set_Coefficient(sPID_2->PID,50,20,20,0);
    PID_Set_limitation(sPID_2,4000,0);
    PID_Set_Ref_Speed(sPID_2,56);
    timer_init();

    while(42);
    return 42;
}

void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void)
{
        PID_Process_Position(pPID_1, NULL, encoders_get_value(pPID_1->encoder_Channel))
		PID_Process_Position(pPID_2, sPID-1, encoders_get_value(pPID_2->encoder_Channel))
        PID_Process_Speed(sPID_2,encoders_get_value(sPID_2->encoder_Channel));

		IFS1bits.T4IF = 0;
}
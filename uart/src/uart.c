//uart.c
#include <ports.h>
#include <p33Fxxxx.h>
#include <stdio.h>
#include "board_cfg.h"
#include "uart.h"

char buff_uart[BUFF_SIZE];
int buff_index;

// send data
void uart_sendc(char c)
{
    while(!UART1_TX_READY); //if buffer is full, wait
    U1TXREG = c;
}

void uart_sendstr(char *s)
{
    int j;
    while (*s)
    {
        uart_sendc(*s);
        s++;
    }
}

void uart_sendmsg(char *m)
{
    uart_sendstr(m);
    uart_sendc('\r');
    uart_sendc('\n');
}

//initialisation de l'UART
void init_uart(void)
{
  //  int j;
    U1MODEbits.STSEL = 0;// 1-stop bit
    U1MODEbits.PDSEL = 0;// No Parity, 8-data bits
    U1MODEbits.ABAUD = 0;// Auto-Baud Disabled
    U1MODEbits.UEN = 0;
    U1MODEbits.BRGH = 0;// Low Speed mode
    U1BRG = BRGVAL;  // BAUD Rate Setting for 9600

    U1STA = 0;
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;
	
    //initialisation des interruptions en reception
    IFS0bits.U1RXIF = 0;      // Reset U1RXIF interrupt flag
    IPC2bits.U1RXIP = 3;      // U1RXIF Interrupt priority level=3
    IEC0bits.U1RXIE = 0;      // Disable U1RX interrupt
}

//*********************//
//**interruption sur U1RX ***//
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    int i;
    // Interrupt Service Routine code goes here */
    IFS0bits.U1RXIF=0;
#if 0
    while(U1STAbits.URXDA)// tant que le buffer est plein
    {
    	char read=U1RXREG;
            buff_uart[buff_index]=read;
            buff_index++;
            if((buff_index > 0) && ((read=='\n') || (read=='\r')))
            {
                buff_uart[buff_index-1]=0;
                is_buffer_ready=1;
                buff_index=0;
            }
    }
#endif
     return;
}

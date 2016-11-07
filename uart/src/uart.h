#ifndef UART_IS_INCLUDED
#define UART_IS_INCLUDED

#define BUFF_SIZE 80
#define BAUDRATE 9600
#define BRGVAL ((FCY/BAUDRATE)/16)-1

#define UART1_TX_READY !U1STAbits.UTXBF
#define UART1_RX_READY U1STAbits.URXDA


void init_uart(void);
void uart_sendc(char c);
void uart_sendstr(char *s);
void uart_sendmsg(char *m);

#endif // UART_IS_INCLUDED

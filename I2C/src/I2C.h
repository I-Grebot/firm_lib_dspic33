#ifndef I2C_IS_INCLUDED
#define I2C_IS_INCLUDED
#include "board_cfg.h"

#define i2c_write   0
#define i2c_read    1

#define i2c_enable do{I2C1CONbits.I2CEN=1;}while(0)
#define i2c_disable do{I2C1CONbits.I2CEN=0;}while(0)
#define i2c_clock_release do{I2C1CONbits.SCLREL=1;}while(0)
#define i2c_clock_hold do{I2C1CONbits.SCLREL=0;}while(0)
#define i2c_10bit_slave do{I2C1CONbits.A10M=1;}while(0)
#define i2c_7bit_slave do{I2C1CONbits.A10M=0;}while(0)
#define i2c_slew_rate_enable do{I2C1CONbits.DISSLW=0;}while(0)
#define i2c_slew_rate_disable do{I2C1CONbits.DISSLW=1;}while(0)
#define i2c_smbus_enable do{I2C1CONbits.SMEN=1;}while(0)
#define i2c_smbus_disable do{I2C1CONbits.SMEN=0;}while(0)
#define i2c_ipmi_enable do{I2C1CONbits.IPMIEN=1;}while(0)
#define i2c_ipmi_disable do{I2C1CONbits.IPMIEN=0;}while(0)
#define i2c_general_call_enable do{I2C1CONbits.GCEN=1;}while(0)
#define i2c_general_call_disable do{I2C1CONbits.GCEN=0;}while(0)
#define i2c_clock_streching_enable do{I2C1CONbits.STREN=1;}while(0)
#define i2c_clock_streching_disable do{I2C1CONbits.STREN=0;}while(0)
#define i2c_send_NACK do{I2C1CONbits.ACKDT=1;I2C1CONbits.ACKEN=1;while(IFS1bits.MI2C1IF==0);i2c_clear_Mflag;}while(0)
#define i2c_send_ACK do{I2C1CONbits.ACKDT=0;I2C1CONbits.ACKEN=1;while(IFS1bits.MI2C1IF==0);i2c_clear_Mflag;}while(0)
#define i2c_receive_enable do{I2C1CONbits.RCEN=1;}while(0)
#define i2c_send_STOP do{I2C1CONbits.PEN=1;while(IFS1bits.MI2C1IF==0);i2c_clear_Mflag;}while(0)
#define i2c_send_REPEATED_START do{I2C1CONbits.RSEN=1;while(IFS1bits.MI2C1IF==0);i2c_clear_Mflag;}while(0)
#define i2c_send_START do{I2C1CONbits.SEN=1;while(IFS1bits.MI2C1IF==0);i2c_clear_Mflag;}while(0)
#define i2c_NACK_received I2C1STATbits.ACKSTAT==1
#define i2c_ACK_received I2C1STATbits.ACKSTAT==0
#define i2c_transmission_in_progress I2C1STATbits.TRSTAT==1
#define i2c_collision_occurs I2C1STATbits.BCL==1
#define i2c_general_call_address_received I2C1STATbits.GCSTAT==1
#define i2c_data_received I2C1STATbits.D_A==1
#define i2c_address_received I2C1STATbits.D_A==0
#define i2c_STOP_bit_detected I2C1STATbits.P==1
#define i2c_START_bit_detected I2C1STATbits.S==1
#define i2c_READ_received I2C1STATbits.R_W==1
#define i2c_WRITE_received I2C1STATbits.R_W==0
#define i2c_receive_buffer_full I2C1STATbits.RBF==1
#define i2c_receive_buffer_empty I2C1STATbits.RBF==0
#define i2c_transmit_buffer_full I2C1STATbits.TBF==1
#define i2c_transmit_buffer_empty I2C1STATbits.TBF==0

#define i2c_clear_Mflag do{IFS1bits.MI2C1IF=0;}while(0)
#define i2c_clear_Sflag do{IFS1bits.SI2C1IF=0;}while(0)

#define i2c_1MHz 0
#define i2c_400kHz 1
#define i2c_100kHz 2

void i2c_init(uint8_t speed);
void i2c_set_mask(uint16_t mask);
void i2c_set_baudrate(uint16_t baudrate);
void i2c_write_char(uint8_t value);
uint8_t i2c_read_char(void);
void i2c_send(uint8_t adr, uint8_t value);
uint8_t i2c_get(uint8_t adr);

#endif // I2C_IS_INCLUDED

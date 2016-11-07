//uart.c
#include "I2C.h"

void i2c_init(uint8_t speed){
    switch(speed)
    {
        case i2c_1MHz:
            i2c_slew_rate_disable;
            i2c_set_baudrate((uint16_t)(FCY*0.87/1000000)-2);
        break;
        case i2c_400kHz:
            i2c_slew_rate_enable;
            i2c_set_baudrate((uint16_t)(FCY*0.948/400000)-2);
        break;
        case i2c_100kHz:
            i2c_slew_rate_disable;
            i2c_set_baudrate((uint16_t)(FCY*0.987/100000)-2);
        default :
        break;
    }
    IEC1bits.MI2C1IE=0;         // disable interrupt;
    IEC1bits.SI2C1IE=0;
    i2c_clear_Mflag;
    i2c_clear_Sflag;


    i2c_enable;
}
void i2c_set_mask(uint16_t mask){
    I2C1MSK = mask&0x03FF;
}

void i2c_set_baudrate(uint16_t baudrate){
    I2C1BRG = baudrate&0x01FF;
}

void i2c_write_char(uint8_t value){
    I2C1TRN = value;
    while(IFS1bits.MI2C1IF==0);     // wait for transmission to be completed
    i2c_clear_Mflag;
}

uint8_t i2c_read_char(void){
    I2C1CONbits.RCEN = 1;
    while(IFS1bits.MI2C1IF==0);     // wait for transmission to be completed
    i2c_clear_Mflag;
    return I2C1RCV;
}

void i2c_send(uint8_t adr, uint8_t value){
    i2c_send_START;
    i2c_write_char(adr|i2c_write);
    while(!i2c_ACK_received);       // wait for slave ACK
    i2c_write_char(value);
    i2c_send_STOP;
}
uint8_t i2c_get(uint8_t adr){
    i2c_send_START;
    i2c_write_char(adr|i2c_read);
    while(!i2c_ACK_received)       // wait for slave ACK
    {
        i2c_send_REPEATED_START;
        i2c_write_char(adr|i2c_read);
    }
    i2c_send_STOP;
}

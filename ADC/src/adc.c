#include "adc.h"

void init_adc1(unsigned mode, uint8_t format, uint8_t clk_source, uint8_t volt_ref)
{
    adc1_stop;                          // disable ADC for configuration

    AD1CON1bits.AD12B = mode;           // select 10bit or 12bit mode
    AD1CON1bits.FORM = format;          // select data output format
    AD1CON1bits.SSRC = clk_source;      // select sample clock source

    AD1CON2bits.VCFG = volt_ref;        // select voltage reference

    AD1CON3bits.ADRC = 0;               // ADC Clock derived from system clock
    AD1CON3bits.ADCS = ADC_clk;         // ADC Conversion Clock : Tad = 64*Tcy
}

void adc1_channel_config(uint8_t channel, uint8_t CHxSA, unsigned CHxNA, uint32_t inputs)
{
    #ifdef __dsPIC33FJ128MC804__
        AD1PCFGL|=(uint16_t) 0x0000FFFF&inputs;
        AD1PCFGH|=(uint16_t) 0x0000FFFF&(inputs>>16);
    #else
        AD1PCFGL|=(uint16_t) 0x0000001F&inputs;
    #endif

    switch(channel)
    {
        case CH0:
            AD1CON2bits.CHPS = CH0;
            AD1CHS0bits.CH0SA=CHxSA;
            AD1CHS0bits.CH0NA=CHxNA;
            break;
        case CH1:
            AD1CON2bits.CHPS = CH0_and_CH1;
            AD1CHS123bits.CH123SA=CHxSA;
            AD1CHS123bits.CH123NA=CHxNA;
            break;
        case CH123:
        default:
            AD1CON2bits.CHPS = All_channels;
            AD1CHS123bits.CH123SA=CHxSA;
            AD1CHS123bits.CH123NA=CHxNA;
            break;
    }  
}

void adc1_alternate_inputs_config(uint8_t CH0SB, unsigned CH0NB, unsigned CH123NB, uint8_t CH123SB)
{
    AD1CON2bits.ALTS=1;
    if(AD1CON1bits.AD12B ==_10bit)                  // if mode 12bit
    {
        AD1CHS123bits.CH123SB=CH123SB;
        AD1CHS123bits.CH123NB=CH123NB;
    }
    AD1CHS0bits.CH0SB=CH0SB;
    AD1CHS0bits.CH0NB=CH0NB;
}

void adc1_dma_config(unsigned buff_mode, uint8_t add_increment )
{
    AD1CON1bits.ADDMABM = buff_mode;
    
    AD1CON2bits.SMPI = add_increment;
}

void adc1_input_scanning_config(uint16_t inputs,uint16_t add_increment)
{
    AD1CON2bits.CSCNA=1;            // enable channel scanning
    AD1CSSL = 0x001F&inputs;
    AD1PCFGL = ~inputs;
    AD1CON2bits.CHPS = CH0;
    AD1CON2bits.SMPI = add_increment;
}

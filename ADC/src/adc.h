//adc.h contain every adc functions
#ifndef _ADC_H_
#define _ADC_H_

// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------
#include <ports.h>
#include <p33Fxxxx.h>
#include <stdio.h>
#include "board_cfg.h"

// -----------------------------------------------------------------------------
// MACROS
// -----------------------------------------------------------------------------
#define adc1_start do{AD1CON1bits.ADON = 1;}while(0)
#define adc1_stop do{AD1CON1bits.ADON = 0;}while(0)
#define adc1_stop_during_idle do{AD1CON1bits.ADSIDL = 1;}while(0)
#define adc1_continue_during_idle do{AD1CON1bits.ADSIDL = 0;}while(0)
#define adc1_simultaneous_sample do{AD1CON1bits.SIMSAM = 1;}while(0)
#define adc1_individual_sample do{AD1CON1bits.SIMSAM = 0;}while(0)
#define adc1_auto_sample do{AD1CON1bits.ASAM = 1;}while(0)
#define adc1_manual_sample do{AD1CON1bits.ASAM = 0;}while(0)
#define adc1_start_sampling do{AD1CON1bits.SAMP = 1;}while(0)
#define adc1_start_conversion do{AD1CON1bits.SAMP = 0;}while(0)
#define adc1_conversion_completed AD1CON1bits.DONE==1
#define adc1_conversion_in_progress AD1CON1bits.DONE==0

#define adc1_enable_interrupt do{IEC0bits.AD1IE = 1;}while(0)
#define adc1_disable_interrupt do{IEC0bits.AD1IE = 0;}while(0)
#define adc1_clear_flag do{IFS0bits.AD1IF = 0;}while(0)

#define adc1_scan_inputs AD1CON2bits.CSCNA=1
#define adc1_do_not_scan_inputs AD1CON2bits.CSCNA=0
#define adc1_second_half_buffer_busy AD1CON2bits.BUFS==1
#define adc1_first_half_buffer_busy AD1CON2bits.BUFS==0

// -----------------------------------------------------------------------------
// DEFINES
// -----------------------------------------------------------------------------
// DMA Buffer Build Mode
#define DMA_buffer_written_in_order 1
#define DMA_buffer_Scatter_Gather   0

// Operation Mode
#define _12bit 1
#define _10bit 0

//Data Output Format bits
#define Signed_fractional   0b11
#define Fractional          0b10
#define Signed_integer      0b01
#define Integer             0b00

//Sample Clock Source
#define Internal_counter    0b111
#define Motor_Control_PWM2  0b101
#define Motor_Control_PWM1  0b011
#define GP_timer_T5_T3      0b100
#define GP_timer_T3_T5      0b010
#define Active_on_INT0      0b001
#define Clearing_sample_bit 0b000

// Converter Voltage Reference Configuration
#define AVDD_AVSS           0b000
#define Vref_AVSS           0b001
#define AVDD_Vref           0b010
#define Vref_Vref           0b011

//Channel Select
#define CH123               0b10
#define All_channels        0b10
#define CH1                 0b01
#define CH0_and_CH1         0b01
#define CH0                 0b00

#define AN0                 0x0001
#define AN1                 0x0002
#define AN2                 0x0004
#define AN3                 0x0008
#define AN4                 0x0010
#define AN5                 0x0020
#define AN6                 0x0040
#define AN7                 0x0080
#define AN8                 0x0100

#if (TAD < 75)      // test if TAD < 75ns
    #error "adc.h Library : TAD must be equal or smaller than 75ns"
#endif

// -----------------------------------------------------------------------------
// PROTOTYPES
// -----------------------------------------------------------------------------
void init_adc1(unsigned mode, uint8_t format, uint8_t clk_source, uint8_t volt_ref);
void adc1_channel_config(uint8_t channel, uint8_t CHxSA, unsigned CHxNA, uint32_t inputs);
void adc1_alternate_inputs_config(uint8_t CH0SB, unsigned CH0NB, unsigned CH123NB, uint8_t CH123SB);
void adc1_dma_config(unsigned buff_mode, uint8_t add_increment);
void adc1_input_scanning_config(uint16_t inputs, uint16_t add_increment);

#endif

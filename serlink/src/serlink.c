/* -----------------------------------------------------------------------------
 * Serlink
 * I-Grebot 1 wire servo-serial link library
 * -----------------------------------------------------------------------------
 * File        : serlink.c
 * Language    : C
 * Author      : Paul M.
 * Date        : 2012-04-22
 * -----------------------------------------------------------------------------
 * Description
 *   The Serlink library provides a 1 wire interface using UART
 *   to control a slave microcontroller tunning analog servo's PWM.
 *   Using packetized informations, the small 5 bytes protocol
 *   allows a great controlling possibilities, since slaves are
 *   dedicated for servo controls.
 * -----------------------------------------------------------------------------
 * Versionning informations
 * Repository: http://svn2.assembla.com/svn/paranoid_android/
 * -----------------------------------------------------------------------------
 * $Rev:: 1170                                                                 $
 * $LastChangedBy:: sebastien.brulais                                          $
 * $LastChangedDate:: 2014-10-31 12:12:12 +0100 (ven., 31 oct. 2014)           $
 * -----------------------------------------------------------------------------
 * Version     Comment                                   Author       Date
 * 1.0         Initial release                           Paul M.      2012-04-22
 * -----------------------------------------------------------------------------
 */

// Board configuration file
#include "board_cfg.h"

// Servo Serial Link header file
#include "serlink.h"

// -----------------------------------------------------------------------------
// MASTER FUNCTIONS
// -----------------------------------------------------------------------------
#ifndef SERLINK_SLAVE

// Initialize Serlink hardware (UART)
void serlink_master_init(void) {

    // UART TX as output
    // Map the PPS output
    SERLINK_TRIS = 0 ;

#ifndef SERLINK_USE_UART2
    SERLINK_RPN       = 3; // UART1_TX
    U1MODEbits.STSEL  = 0; // 1 stop bit
    U1MODEbits.PDSEL  = 0; // No parity, 8 data bit
    U1MODEbits.ABAUD  = 0; // Auto-baud disabled
    U1MODEbits.BRGH   = 0; // Low-speed mode
    U1BRG = SERLINK_BRG ;  // Baud-rate config
    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN   = 1; // Enable TX on UART
#else
    SERLINK_RPN       = 5; // UART2_TX
    U2MODEbits.STSEL  = 0; // 1 stop bit
    U2MODEbits.PDSEL  = 0; // No parity, 8 data bit
    U2MODEbits.ABAUD  = 0; // Auto-baud disabled
    U2MODEbits.BRGH   = 0; // Low-speed mode
    U2BRG = SERLINK_BRG  ; // Baud-rate config
    U2MODEbits.UARTEN = 1;// Enable UART
    U2STAbits.UTXEN   = 1;  // Enable TX on UART
#endif
    
}

void serlink_master_send(serlink_packet_t* tx_packet) {
    
    // Also compute checksum at the same time
    uint8_t checksum;

#ifndef SERLINK_USE_UART2
    // Send Synchro byte header
    while (U1STAbits.UTXBF) ;
    U1TXREG = SERLINK_SYNC_HEADER;

    // Send control byte
    while (U1STAbits.UTXBF) ;
    checksum = SERLINK_MAKE_CTRL(tx_packet->addr, tx_packet->latch, tx_packet->cmd);
    U1TXREG = checksum;

    // Send data bytes, MSB first
    while (U1STAbits.UTXBF) ;
    checksum += tx_packet->value >> 8 ;
    U1TXREG   = tx_packet->value >> 8 ;
    while (U1STAbits.UTXBF) ; 
    checksum += tx_packet->value & 0xFF;
    U1TXREG   = tx_packet->value & 0xFF;

    // Send checksum byte
    while (U1STAbits.UTXBF);
    U1TXREG = ~checksum;
#else
    // Send Synchro byte header
    while (U2STAbits.UTXBF) ;
    U2TXREG = SERLINK_SYNC_HEADER;

    // Send control byte
    while (U2STAbits.UTXBF) ;
    checksum = SERLINK_MAKE_CTRL(tx_packet->addr, tx_packet->latch, tx_packet->cmd);
    U2TXREG = checksum;

    // Send data bytes, MSB first
    while (U2STAbits.UTXBF) ;
    checksum += tx_packet->value >> 8 ;
    U2TXREG   = tx_packet->value >> 8 ;
    while (U2STAbits.UTXBF) ;
    checksum += tx_packet->value & 0xFF;
    U2TXREG   = tx_packet->value & 0xFF;

    // Send checksum byte
    while (U2STAbits.UTXBF);
    U2TXREG = ~checksum;
#endif
    
}

// -----------------------------------------------------------------------------
// SLAVE FUNCTIONS
// -----------------------------------------------------------------------------
#else

// Slave init
void serlink_slave_init(void) {

    // Initialize UART wire as input
    SERLINK_TRIS = 1;

    // Configure UART for RX
    // With FOSC = 32MHz, achievable baudrates range between :
    // baudrate = FOSC/(16*(BRG+1))
    // min baudrate = 32e6/(16 * 2^16) = 0.031 kbps
    // max baudrate = 32e6/(16 *    1) =     2 Mbps
    BAUDCONbits.BRG16 = 0; //     BRG Base clock = FOSC/16
    TXSTAbits.BRGH    = 1; // and BRG ABD clock  = FOSC/128
    RCSTAbits.SPEN    = 1; // Enable Serial Port
    TXSTAbits.SYNC    = 0; // Select asynchronous mode
    RCSTAbits.CREN    = 1; // Enable UART RX

    // Configure interrupt
  //PIR1bits.RCIF = 0; // Clear RX flag
    PIE1bits.RCIE = 1; // Enable RX interrupt

    // Setup auto-baud for 1st RX
    BAUDCONbits.ABDEN = 1;
    
}

// Validate packet bytes against FEC value using a simple checksum
// Returns difference of checksums (0 if no error detected)
uint8_t serlink_validate_fec(uint8_t packet_bytes[SERLINK_PACKET_LEN], uint8_t fec) {

    uint8_t idx;
    uint8_t checksum;

    for(idx=0, checksum=0; idx<SERLINK_PACKET_LEN; idx++)
        checksum+=packet_bytes[idx];

    return fec - (~checksum);
}

// Slave receive
uint8_t serlink_slave_receive(serlink_packet_t* rx_packet) {

    uint8_t rx_byte[SERLINK_PACKET_LEN];
    uint8_t fec;
    uint8_t error_flag;

    // Disable RX interrupt : continuous bytes are expected
    PIE1bits.RCIE = 0;

    // Init error flag
    error_flag = 0;

    // Ensure that auto-baud-rate sequence is finished
    while(BAUDCONbits.ABDEN);
    
    // Receive sync after auto-baud-rate took place.
    // Content should be discarded.
    rx_byte[0] = RCREG;

    // Get Control byte and check errors
    while(!PIR1bits.RCIF);
    rx_byte[0] = RCREG;
    if(RCSTAbits.OERR || RCSTAbits.FERR) error_flag |= 1;

    // Get Value MSW byte and check errors
    while(!PIR1bits.RCIF);
    rx_byte[1] = RCREG;
    if(RCSTAbits.OERR || RCSTAbits.FERR) error_flag |= 1;

    // Get Value LSW byte and check errors
    while(!PIR1bits.RCIF);
    rx_byte[2] = RCREG;
    if(RCSTAbits.OERR || RCSTAbits.FERR) error_flag |= 1;

    // Get FEC byte and check errors
    while(!PIR1bits.RCIF);
    fec = RCREG;
    if(RCSTAbits.OERR || RCSTAbits.FERR) error_flag |= 1;

    // If HW error occurred, clear error flags by clearing CREN
    if(error_flag & 1) {
        RCSTAbits.CREN = 0;
        while(RCSTAbits.OERR || RCSTAbits.FERR);
        RCSTAbits.CREN = 1;
    
    // No HW error occured
    } else {

        // If received data are not valid, set error flag
        if(serlink_validate_fec(rx_byte, fec)) error_flag |= 2;

        // Everything went smoothly, match parameters
        else {

            // Retrieve control packet content
            rx_packet->addr  = SERLINK_GET_ADDR(rx_byte[0]);
            rx_packet->latch = SERLINK_GET_LATCH(rx_byte[0]);
            rx_packet->cmd   = SERLINK_GET_CMD(rx_byte[0]);

            // Retrieve value content
            rx_packet->value = ((uint16_t) rx_byte[1])<<8 | rx_byte[2];
        }
    }

    // Setting auto-baud for next packet
    BAUDCONbits.ABDEN = 1;
    
    // Enable back RX interrupts
    PIE1bits.RCIE = 1;
    
    return error_flag;
}

#endif


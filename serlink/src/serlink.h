/* -----------------------------------------------------------------------------
 * Serlink
 * I-Grebot 1 wire servo-serial link library
 * -----------------------------------------------------------------------------
 * File        : serlink.h
 * Language    : C
 * Author      : Paul M.
 * Date        : 2012-04-22
 * -----------------------------------------------------------------------------
 * Description
 *   Serlink library main header.
 *   See module's header for complete description.
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

#ifndef _SERLINK_H_
#define _SERLINK_H_

//------------------------------------------------------------------------------
// LIBRARY CONSTANTS
//------------------------------------------------------------------------------

// Serlink packets:
// <SYNC> <CTRL> <VALUE_H> <VALUE_L> <FER>
// SYNC     : A synchronization byte used to improve baud-rate detection
//              and to identify the start of a packet.
// CTRL     : Control byte (see bellow for possible commands)
// VALUE_H  : MSW of value to transmit
// VALUE_L  : LSW of value to transmit
// FEC      : Forward-Error-Control bye to improve link error recovery

// Commands:
// CTRL <7:4> (4 bits)  : Servo address / Up to 15 servos
//                        0b0000 corresponds to the broadcast address,
//                               it'll affect all servos.
// CTRL <3> (1 bit)     : Latch control
//                        0b0 : Programmed command takes action immediatly
//                        0b1 : Programmed command is stored
// CTRL <2:0> (3 bits)  : Command word
//                         ENA  - Enable/Disable servos (do not use address field)
//                         POS  - Write servo position
//                         SPD  - Write servo maximum speed
//                         ACC  - Write servo maximum acceleration
//                         TRIG - Trig stored POS, SPD and ACC values
//                         SEL  - Slaves selection : enable/disable
//                                       completely a slave depending on the
//                                       <VALUE_H:VALUE_L> bits (1 = enabled)

// A packet length in bytes, without sync and fec
#define SERLINK_PACKET_LEN 3

#define SERLINK_ADDR_BROADCAST 0

// Commands
#define SERLINK_CMD_ENA  0b000
#define SERLINK_CMD_POS  0b001
#define SERLINK_CMD_SPD  0b010
#define SERLINK_CMD_ACC  0b011
#define SERLINK_CMD_TRIG 0b100
#define SERLINK_CMD_SEL  0b101

// Synchronization packet, also used for auto-baud-rate feature
#define SERLINK_SYNC_HEADER 0x55

// Control byte manipulation macros
#define SERLINK_MAKE_CTRL(_a, _l, _c) ( ((_a) & 0x0F)<<4 | ((_l) & 0x01)<<3 | ((_c) & 0x07) )
#define SERLINK_GET_ADDR(_x)  ( ((_x) >> 4) & 0x0F )
#define SERLINK_GET_LATCH(_x) ( ((_x) >> 3) & 0x01 )
#define SERLINK_GET_CMD(_x)   (        (_x) & 0x07 )

// Define Serlink baudrate (used for master side)
// 5 bytes @115.2 kbps => 415 us for a TX
// 1 servo pulse = maximum 2000 us
#define SERLINK_BAUDRATE 115200

// Compute actual UART1 BRG value (Low-speed baudrate)
#define SERLINK_BRG ((FCY/16/SERLINK_BAUDRATE)-1)

// Packet structure
// ----------------

// SYNC and FEC bytes do not appear as they are related to the
// hardware link and would be useless in upper application layers.
typedef struct {
    uint8_t  addr  : 4;  // Servo address
    uint8_t  latch : 1;  // Latch command
    uint8_t  cmd   : 3;  // Command
    uint16_t value    ;  // 16 bit value
}serlink_packet_t;


// Prototypes for master functions
// -------------------------------
#ifndef SERLINK_SLAVE
void serlink_master_init(void);
void serlink_master_send(serlink_packet_t* tx_packet);

// Prototypes for slave side
// -------------------------
#else
void serlink_slave_init(void);
uint8_t serlink_validate_fec(uint8_t packet_bytes[SERLINK_PACKET_LEN], uint8_t fec);
uint8_t serlink_slave_receive(serlink_packet_t* rx_packet);

#endif // SERLINK_SLAVE


#endif // _SERLINK_H_

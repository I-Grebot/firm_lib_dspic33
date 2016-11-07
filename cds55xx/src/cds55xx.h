#ifndef _CDS55XX_H
#define _CDS55XX_H

// Register MAP is defined in a specific header
#include "cds55xx_regmap.h"

// -----------------------------------------------------------------------------
// CDS55XX Types definition
// -----------------------------------------------------------------------------

// Maximum number of parameters in a packet
// (either instruction or status)
#define CDS55XX_MAX_PARAM  32

// RX/TX States for the switch handler
#define CDS55XX_TX 1
#define CDS55XX_RX 0

// CDS55XX config
// Define hardware handlers (function pointers)
typedef struct {

    // Switch to TX/RX the half-duplex link
    void (* hw_switch)(unsigned char);

    // Send a byte and return error if not successful
    unsigned char (* hw_send_byte)(unsigned char);

    // Receive a byte and return error if not successful
    unsigned char (* hw_receive_byte) (unsigned char*);

    // Flush the receiver (used to make sure its clean before
    // starting to receive actual datas)
    void (* hw_flush) (void);

    // Return level that needs to be remembered to know if
    // something has to be expected.
    unsigned char return_level;

    // Return delay also has to be remembered to set an
    // apropriate timeout.
    unsigned char return_delay;
    
} cds55xx_cfg_t;

// CDS55XX packets
// Double-byte header is not represented,
// Instruction and status packets only depends on their contents:
//  - An instruction for the instruction packets (write, read, action etc.)
//  - Servo's status for the status packets
typedef struct {
    unsigned char id;
    unsigned char length;
    unsigned char content;   // This is either an instruction or a status
    unsigned char parameters[CDS55XX_MAX_PARAM];
    unsigned char checksum;
} cds55xx_packet_t;

// -----------------------------------------------------------------------------
// Errors definitions
// -----------------------------------------------------------------------------

// CDS550 error flags (see datasheet)
#define CDS55XX_ERROR_VOLTAGE     0x01
#define CDS55XX_ERROR_POSITION    0x02
#define CDS55XX_ERROR_OVERHEAT    0x04
#define CDS55XX_ERROR_RANGE       0x08
#define CDS55XX_ERROR_CHECKSUM    0x10
#define CDS55XX_ERROR_OVERLOAD    0x20
#define CDS55XX_ERROR_INSTRUCTION 0x40

// UART status error flags
#define CDS55XX_ERROR_UART_TIMEOUT  0x0100  // Status packet timed-out
#define CDS55XX_ERROR_UART_HEADER   0x0200  // Incorrect status packet header

// Application status errors masks definition
#define CDS55XX_ERROR_APP_ID        0x0400  // Status packet ID does not match
#define CDS55XX_ERROR_APP_LENGTH    0x0800  // Status packet length doesnt match
#define CDS55XX_ERROR_APP_CHECKSUM  0x1000  // Status packet checksum is wrong

// -----------------------------------------------------------------------------
// Instructions definitions
// -----------------------------------------------------------------------------

// Instructions definition
#define CDS55XX_HEADER            0xFF
#define CDS55XX_INS_PING          0x01
#define CDS55XX_INS_READ          0x02
#define CDS55XX_INS_WRITE         0x03
#define CDS55XX_INS_REG_WRITE     0x04
#define CDS55XX_INS_ACTION        0x05
#define CDS55XX_INS_RESET         0x06
#define CDS55XX_INS_SYNC_WRITE    0x83

// ID definitions
#define CDS55XX_ID_BROADCAST      0xFE
#define CDS55XX_ID_IMPOSSIBLE     0xFF

// Status answer level definition
#define CDS55XX_STATUS_NO_ANSWER   0x00
#define CDS55XX_STATUS_READ_ONLY   0x01
#define CDS55XX_STATUS_EVERYTHING  0x02

// -----------------------------------------------------------------------------
// Maximum values allowed
// -----------------------------------------------------------------------------

#define CDS55XX_MAX_POSITION          0x03FF  // 300 °
#define CDS55XX_MAX_SPEED             0x03FF  // 62 RPM
#define CDS55XX_MAX_ACCELERATION      0x03FF  // ?? RPM / sec
#define CDS55XX_MAX_PUNCH             0x03FF  // ??

// -----------------------------------------------------------------------------
// Prototypes
// -----------------------------------------------------------------------------

// UART Layer functions
void cds55xx_send_packet(cds55xx_packet_t* packet);
unsigned int cds55xx_receive_packet(cds55xx_packet_t* packet);
unsigned char cds55xx_compute_checksum(cds55xx_packet_t* packet);

// INSTRUCTION Layer functions
unsigned int cds55xx_get_status(cds55xx_packet_t* instruction_packet,
                                cds55xx_packet_t* status_packet,
                                unsigned char expected_param_length);
unsigned int cds55xx_ping(unsigned char id);
unsigned int cds55xx_write(unsigned char id, unsigned char address, unsigned char* parameters,
                           unsigned char nb_param, unsigned char registered);
unsigned int cds55xx_read(unsigned char id, unsigned char address, unsigned char* datas, unsigned char nb_data);
void cds55xx_action(void);
unsigned int cds55xx_reset(unsigned char id);

#endif // _CDS55XX_H



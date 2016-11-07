#ifndef _XL_320_H
#define _XL_320_H

//#include "board_cfg.h"

// Register MAP is defined in a specific header
#include "xl_320_regmap.h"
#include <stdint.h>

// -----------------------------------------------------------------------------
// XL-320 Types definition
// -----------------------------------------------------------------------------

// Maximum number of parameters in a packet
// (either instruction or status)
#define XL_320_MAX_PARAM  32

// RX/TX States for the switch handler
#define XL_320_TX 1
#define XL_320_RX 0

// XL-320 config
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
    
} xl_320_cfg_t;

// XL-320 packets
// Double-byte header is not represented,
// Instruction and status packets only depends on their contents:
//  - An instruction for the instruction packets (write, read, action etc.)
//  - Servo's status for the status packets
typedef struct {
    uint8_t id;
    uint8_t length;
    uint8_t content;   // This is either an instruction or a status
    uint8_t parameters[XL_320_MAX_PARAM];
    uint16_t checksum;
} xl_320_packet_t;

// -----------------------------------------------------------------------------
// Errors definitions
// -----------------------------------------------------------------------------

// UART status error flags
#define XL_320_ERROR_UART_TIMEOUT  0x0100  // Status packet timed-out
#define XL_320_ERROR_UART_HEADER   0x0200  // Incorrect status packet header

// Application status errors masks definition
#define XL_320_ERROR_APP_ID        0x0400  // Status packet ID does not match
#define XL_320_ERROR_APP_LENGTH    0x0800  // Status packet length doesnt match
#define XL_320_ERROR_APP_CHECKSUM  0x1000  // Status packet checksum is wrong

// XL-320 error flags (see datasheet)
#define XL_320_ERROR_INPUT_VOLTAGE  0x04    // Voltage is out of operational voltage range
#define XL_320_ERROR_OVER_HEATING   0x02    // Temperature is out of operational temperature range
#define XL_320_ERROR_OVERLOAD       0x01    // Motor cannot output max load due to load being applied continouosly

// -----------------------------------------------------------------------------
// Instructions definitions
// -----------------------------------------------------------------------------

// Instructions definition
#define XL_320_HEADER            0xFF
#define XL_320_HEADER_2          0xFD
#define XL_320_INS_PING          0x01
#define XL_320_INS_READ          0x02
#define XL_320_INS_WRITE         0x03
#define XL_320_INS_REG_WRITE     0x04
#define XL_320_INS_ACTION        0x05
#define XL_320_INS_RESET         0x06
#define XL_320_REBOOT            0x08
#define XL_320_INS_SYNC_READ     0x82
#define XL_320_INS_SYNC_WRITE    0x83
#define XL_320_INS_BULK_READ     0x92
#define XL_320_INS_BULK_WRITE    0x93
#define XL_320_STATUS            0x55

// ID definitions
#define XL_320_ID_BROADCAST      0xFE
#define XL_320_ID_IMPOSSIBLE     0xFF

// Status answer level definition
#define XL_320_STATUS_NO_ANSWER   0x00
#define XL_320_STATUS_READ_ONLY   0x01
#define XL_320_STATUS_EVERYTHING  0x02

// -----------------------------------------------------------------------------
// Maximum values allowed
// -----------------------------------------------------------------------------

#define XL_320_MAX_POSITION          0x03FF  // 300 °
#define XL_320_MAX_SPEED             0x03FF  // 62 RPM
#define XL_320_MAX_ACCELERATION      0x03FF  // ?? RPM / sec
#define XL_320_MAX_PUNCH             0x03FF  // ??

// -----------------------------------------------------------------------------
// Registers meanings
// -----------------------------------------------------------------------------

// Fields definition for the baudrate register
#define XL_320_BAUDRATE_9600BPS     0
#define XL_320_BAUDRATE_57600BPS    1
#define XL_320_BAUDRATE_115200BPS   2
#define XL_320_BAUDRATE_1MBPS       3

// LEDs color
#define XL_320_LED_OFF    0
#define XL_320_LED_RED    1
#define XL_320_LED_GREEN  2
#define XL_320_LED_BLUE   4
#define XL_320_LED_YELLOW  (XL_320_LED_RED   | XL_320_LED_GREEN)
#define XL_320_LED_CYAN    (XL_320_LED_GREEN | XL_320_LED_BLUE)
#define XL_320_LED_MAGENTA (XL_320_LED_RED   | XL_320_LED_BLUE)
#define XL_320_LED_WHITE   (XL_320_LED_RED   | XL_320_LED_GREEN | XL_320_LED_BLUE)

// -----------------------------------------------------------------------------
// Prototypes
// -----------------------------------------------------------------------------

void dservo_start_timout(uint16_t value);
uint8_t dservo_timout_is_running(void);
void dservo_instruction_delay(void);


// UART Layer functions
void xl_320_send_sample_packet();
void xl_320_send_packet(xl_320_packet_t* packet);
unsigned int xl_320_receive_packet(xl_320_packet_t* packet);
uint16_t xl_320_compute_checksum(xl_320_packet_t* packet);

// INSTRUCTION Layer functions
unsigned int xl_320_get_status(xl_320_packet_t* instruction_packet,
                                xl_320_packet_t* status_packet,
                                unsigned char expected_param_length);
unsigned int xl_320_ping(unsigned char id);
unsigned int xl_320_write(unsigned char id, unsigned char address, unsigned char* parameters,
                           unsigned char nb_param, unsigned char registered);
unsigned int xl_320_read(unsigned char id, unsigned char address, unsigned char* datas, unsigned char nb_data);
void xl_320_action(void);
unsigned int xl_320_reset(unsigned char id);


void xl_320_set_id(unsigned char id) ;
void xl_320_set_baudrate(uint8_t baudrate);
unsigned int xl_320_set_return_delay(unsigned char id, unsigned char return_delay);
unsigned int xl_320_set_temp_limit(unsigned char id, unsigned char temp_limit);
unsigned int xl_320_set_min_voltage(unsigned char id, unsigned char min_voltage);
unsigned int xl_320_set_max_voltage(unsigned char id, unsigned char max_voltage);
//unsigned int xl_320_set_alarm_shutdown(unsigned char id, unsigned char alarm_shutdown);
unsigned int xl_320_set_cw_limit(unsigned char id, unsigned int cw_limit);
unsigned int xl_320_set_ccw_limit(unsigned char id, unsigned int ccw_limit);
unsigned int xl_320_set_goal_torque(unsigned char id, unsigned int torque_limit);
unsigned int xl_320_set_status_return(unsigned char id, unsigned char status_return);
//unsigned int xl_320_set_torque_en(unsigned char id, unsigned char torque_en, unsigned char registered);
void xl_320_set_led(unsigned char id, unsigned char led);
void xl_320_set_position(uint8_t id, uint16_t position);

void xl_320_set_torque_en(unsigned char id, unsigned char torque_en);
void xl_320_set_alarm_shutdown(uint8_t id, uint8_t shutdown);

//unsigned int xl_320_set_position(unsigned char id, unsigned int position, unsigned char registered);
//unsigned int xl_320_set_speed(unsigned char id, unsigned int speed, unsigned char registered);
void xl_320_set_speed(unsigned char id, unsigned int speed);

unsigned int xl_320_set_punch(unsigned char id, unsigned int punch, unsigned char registered);
unsigned int xl_320_get_temperature(unsigned char id, unsigned char *temp);
unsigned int xl_320_get_voltage(unsigned char id, unsigned char *voltage);
unsigned int xl_320_get_moving(unsigned char id, unsigned char *moving);
unsigned int xl_320_get_position(unsigned char id, unsigned int *position);
unsigned int xl_320_get_speed(unsigned char id, unsigned int *speed);
unsigned int xl_320_get_load(unsigned char id, unsigned int *load);
unsigned int xl_320_set_motion_reg(unsigned char id, unsigned int position,
                                           unsigned int speed, unsigned int torque);
unsigned int xl_320_set_motions(unsigned char *ids, unsigned int *positions,
                    unsigned int *speeds, unsigned char *accs, unsigned char *dccs,
                    unsigned char nb_id);

void xl_320_set_return_level(uint8_t level);

#endif // _XL_320_H



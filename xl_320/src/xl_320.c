#include "xl_320.h"

// The XL320 must be declared as a global outside of this module.
// This is done in order to ease the manipulation of the different
// high-level functions (avoiding to give the current config as
// an argument).
extern xl_320_cfg_t xl_320_cfg;

// -----------------------------------------------------------------------------
// First layer: handles UART data
// -----------------------------------------------------------------------------

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

void xl_320_send_sample_packet()
{

    static uint8_t led=1;
    unsigned short crc16;
    uint8_t buffer[]={XL_320_HEADER,XL_320_HEADER,XL_320_HEADER_2,0,0xFE,0x06,0x00,0x03,0x19,0x00,led};
    // Set uart2 half duplex to TX mode
    xl_320_cfg.hw_switch(XL_320_TX);

    crc16 = update_crc(0, buffer,11);

    xl_320_cfg.hw_send_byte(XL_320_HEADER); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(XL_320_HEADER); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(XL_320_HEADER_2); // Note: this is not the same as AX-12
    xl_320_cfg.hw_send_byte(0x00); //Reserved
    xl_320_cfg.hw_send_byte(0xFE); // XL-320 id
    xl_320_cfg.hw_send_byte(0x06); // packet length
    xl_320_cfg.hw_send_byte(0x00); // packet length MSB assuming length is <255
    xl_320_cfg.hw_send_byte(0x03); // content (instruction or status)
    xl_320_cfg.hw_send_byte(0x19); // param1
    xl_320_cfg.hw_send_byte(0x00); // param2
    xl_320_cfg.hw_send_byte(led); // param3
    xl_320_cfg.hw_send_byte(crc16 & 0xFF); // CRC_L
    xl_320_cfg.hw_send_byte((crc16>>8)&0xFF); // CRC_H

    // Get back to RX mode
    xl_320_cfg.hw_switch(XL_320_RX);

    led++;
    if(led>=7)
    {
        led=1;
    }
}

void xl_320_set_led(uint8_t id , uint8_t led)
{
    unsigned short crc16;
    uint8_t buffer[]={XL_320_HEADER,XL_320_HEADER,XL_320_HEADER_2,0,id,0x06,0x00,0x03,0x19,0x00,led};
    // Set uart2 half duplex to TX mode
    xl_320_cfg.hw_switch(XL_320_TX);

    crc16 = update_crc(0, buffer,11);

    xl_320_cfg.hw_send_byte(buffer[0]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[1]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[2]); // Note: this is not the same as AX-12
    xl_320_cfg.hw_send_byte(buffer[3]); //Reserved
    xl_320_cfg.hw_send_byte(buffer[4]); // XL-320 id
    xl_320_cfg.hw_send_byte(buffer[5]); // packet length
    xl_320_cfg.hw_send_byte(buffer[6]); // packet length MSB assuming length is <255
    xl_320_cfg.hw_send_byte(buffer[7]); // content (instruction or status)
    xl_320_cfg.hw_send_byte(buffer[8]); // param1
    xl_320_cfg.hw_send_byte(buffer[9]); // param2
    xl_320_cfg.hw_send_byte(buffer[10]); // param3
    xl_320_cfg.hw_send_byte(crc16 & 0xFF); // CRC_L
    xl_320_cfg.hw_send_byte((crc16>>8)&0xFF); // CRC_H

    // Get back to RX mode
    xl_320_cfg.hw_switch(XL_320_RX);
}

void xl_320_set_id(uint8_t id)
{
    unsigned short crc16;
    uint8_t buffer[]={XL_320_HEADER,XL_320_HEADER,XL_320_HEADER_2,0,0xFE,0x06,0x00,0x03,0x03,00,id};
    
    // Set uart2 half duplex to TX mode
    xl_320_cfg.hw_switch(XL_320_TX);

    crc16 = update_crc(0, buffer,11);

    xl_320_cfg.hw_send_byte(buffer[0]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[1]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[2]); // Note: this is not the same as AX-12
    xl_320_cfg.hw_send_byte(buffer[3]); //Reserved
    xl_320_cfg.hw_send_byte(buffer[4]); // XL-320 id
    xl_320_cfg.hw_send_byte(buffer[5]); // packet length
    xl_320_cfg.hw_send_byte(buffer[6]); // packet length MSB assuming length is <255
    xl_320_cfg.hw_send_byte(buffer[7]); // content (instruction or status)
    xl_320_cfg.hw_send_byte(buffer[8]); // param1
    xl_320_cfg.hw_send_byte(buffer[9]); // param2
    xl_320_cfg.hw_send_byte(buffer[10]); // param3
    xl_320_cfg.hw_send_byte(crc16 & 0xFF); // CRC_L
    xl_320_cfg.hw_send_byte((crc16>>8)&0xFF); // CRC_H

    // Get back to RX mode
    xl_320_cfg.hw_switch(XL_320_RX);}

void xl_320_set_return_level(uint8_t level)
{
    unsigned short crc16;
    uint8_t buffer[]={XL_320_HEADER,XL_320_HEADER,XL_320_HEADER_2,0,XL_320_ID_BROADCAST,0x06,0x00,0x03,0x11,00,level};
    // Set uart2 half duplex to TX mode
    xl_320_cfg.hw_switch(XL_320_TX);
    crc16 = update_crc(0, buffer,11);
    xl_320_cfg.hw_send_byte(buffer[0]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[1]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[2]); // Note: this is not the same as AX-12
    xl_320_cfg.hw_send_byte(buffer[3]); //Reserved
    xl_320_cfg.hw_send_byte(buffer[4]); // XL-320 id
    xl_320_cfg.hw_send_byte(buffer[5]); // packet length
    xl_320_cfg.hw_send_byte(buffer[6]); // packet length MSB assuming length is <255
    xl_320_cfg.hw_send_byte(buffer[7]); // content (instruction or status)
    xl_320_cfg.hw_send_byte(buffer[8]); // param1
    xl_320_cfg.hw_send_byte(buffer[9]); // param2
    xl_320_cfg.hw_send_byte(buffer[10]); // param3
    xl_320_cfg.hw_send_byte(crc16 & 0xFF); // CRC_L
    xl_320_cfg.hw_send_byte((crc16>>8)&0xFF); // CRC_H

    // Get back to RX mode
    xl_320_cfg.hw_switch(XL_320_RX);

}

void xl_320_set_baudrate(uint8_t baudrate)
{
    unsigned short crc16;
    uint8_t buffer[]={XL_320_HEADER,XL_320_HEADER,XL_320_HEADER_2,0,0xFE,0x06,0x00,0x03,0x04,00,baudrate};
    // Set uart2 half duplex to TX mode
    xl_320_cfg.hw_switch(XL_320_TX);
    crc16 = update_crc(0, buffer,11);
    xl_320_cfg.hw_send_byte(buffer[0]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[1]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[2]); // Note: this is not the same as AX-12
    xl_320_cfg.hw_send_byte(buffer[3]); //Reserved
    xl_320_cfg.hw_send_byte(buffer[4]); // XL-320 id
    xl_320_cfg.hw_send_byte(buffer[5]); // packet length
    xl_320_cfg.hw_send_byte(buffer[6]); // packet length MSB assuming length is <255
    xl_320_cfg.hw_send_byte(buffer[7]); // content (instruction or status)
    xl_320_cfg.hw_send_byte(buffer[8]); // param1
    xl_320_cfg.hw_send_byte(buffer[9]); // param2
    xl_320_cfg.hw_send_byte(buffer[10]); // param3
    xl_320_cfg.hw_send_byte(crc16 & 0xFF); // CRC_L
    xl_320_cfg.hw_send_byte((crc16>>8)&0xFF); // CRC_H

    // Get back to RX mode
    xl_320_cfg.hw_switch(XL_320_RX);

}

void xl_320_set_alarm_shutdown(uint8_t id, uint8_t shutdown)
{
    unsigned short crc16;
    uint8_t buffer[]={XL_320_HEADER,XL_320_HEADER,XL_320_HEADER_2,0,id, 0x06,0x00,0x03,XL_320_ADDR_ALARM_SHUTDOWN,00,shutdown};
    // Set uart2 half duplex to TX mode
    xl_320_cfg.hw_switch(XL_320_TX);
    crc16 = update_crc(0, buffer,11);
    xl_320_cfg.hw_send_byte(buffer[0]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[1]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[2]); // Note: this is not the same as AX-12
    xl_320_cfg.hw_send_byte(buffer[3]); //Reserved
    xl_320_cfg.hw_send_byte(buffer[4]); // XL-320 id
    xl_320_cfg.hw_send_byte(buffer[5]); // packet length
    xl_320_cfg.hw_send_byte(buffer[6]); // packet length MSB assuming length is <255
    xl_320_cfg.hw_send_byte(buffer[7]); // content (instruction or status)
    xl_320_cfg.hw_send_byte(buffer[8]); // param1
    xl_320_cfg.hw_send_byte(buffer[9]); // param2
    xl_320_cfg.hw_send_byte(buffer[10]); // param3
    xl_320_cfg.hw_send_byte(crc16 & 0xFF); // CRC_L
    xl_320_cfg.hw_send_byte((crc16>>8)&0xFF); // CRC_H

    // Get back to RX mode
    xl_320_cfg.hw_switch(XL_320_RX);

}

void xl_320_set_torque_en(unsigned char id, unsigned char torque_en) {

    unsigned short crc16;
    uint8_t buffer[]={XL_320_HEADER,XL_320_HEADER,XL_320_HEADER_2,0,id, 0x06,0x00,0x03,XL_320_ADDR_TORQUE_EN,00,torque_en};
    // Set uart2 half duplex to TX mode
    xl_320_cfg.hw_switch(XL_320_TX);
    crc16 = update_crc(0, buffer,11);
    xl_320_cfg.hw_send_byte(buffer[0]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[1]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[2]); // Note: this is not the same as AX-12
    xl_320_cfg.hw_send_byte(buffer[3]); //Reserved
    xl_320_cfg.hw_send_byte(buffer[4]); // XL-320 id
    xl_320_cfg.hw_send_byte(buffer[5]); // packet length
    xl_320_cfg.hw_send_byte(buffer[6]); // packet length MSB assuming length is <255
    xl_320_cfg.hw_send_byte(buffer[7]); // content (instruction or status)
    xl_320_cfg.hw_send_byte(buffer[8]); // param1
    xl_320_cfg.hw_send_byte(buffer[9]); // param2
    xl_320_cfg.hw_send_byte(buffer[10]); // param3
    xl_320_cfg.hw_send_byte(crc16 & 0xFF); // CRC_L
    xl_320_cfg.hw_send_byte((crc16>>8)&0xFF); // CRC_H

    // Get back to RX mode
    xl_320_cfg.hw_switch(XL_320_RX);


}



void xl_320_set_position(uint8_t id, uint16_t position)
{
    unsigned short crc16;
    uint8_t pos_LSB=position&0xFF;
    uint8_t pos_MSB=((position>>8)&0xFF);
    uint8_t buffer[]={XL_320_HEADER,XL_320_HEADER,XL_320_HEADER_2,0,id,0x07,0x00,0x03,XL_320_ADDR_GOAL_POS_L,00,pos_LSB,pos_MSB};
    // Set uart2 half duplex to TX mode
    xl_320_cfg.hw_switch(XL_320_TX);
    crc16 = update_crc(0, buffer,12);
    xl_320_cfg.hw_send_byte(buffer[0]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[1]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[2]); // Note: this is not the same as AX-12
    xl_320_cfg.hw_send_byte(buffer[3]); //Reserved
    xl_320_cfg.hw_send_byte(buffer[4]); // XL-320 id
    xl_320_cfg.hw_send_byte(buffer[5]); // packet length
    xl_320_cfg.hw_send_byte(buffer[6]); // packet length MSB assuming length is <255
    xl_320_cfg.hw_send_byte(buffer[7]); // content (instruction or status)
    xl_320_cfg.hw_send_byte(buffer[8]); // param1
    xl_320_cfg.hw_send_byte(buffer[9]); // param2
    xl_320_cfg.hw_send_byte(buffer[10]); // param3
    xl_320_cfg.hw_send_byte(buffer[11]); // param4
    xl_320_cfg.hw_send_byte(crc16 & 0xFF); // CRC_L
    xl_320_cfg.hw_send_byte((crc16>>8)&0xFF); // CRC_H

    // Get back to RX mode
    xl_320_cfg.hw_switch(XL_320_RX);

}



void xl_320_set_speed(unsigned char id, unsigned int speed) {
    unsigned short crc16;
    uint8_t pos_LSB=speed&0xFF;
    uint8_t pos_MSB=((speed>>8)&0xFF);
    uint8_t buffer[]={XL_320_HEADER,XL_320_HEADER,XL_320_HEADER_2,0,id,0x07,0x00,0x03,XL_320_ADDR_GOAL_VELOCITY_L,00,pos_LSB,pos_MSB};
    // Set uart2 half duplex to TX mode
    xl_320_cfg.hw_switch(XL_320_TX);
    crc16 = update_crc(0, buffer,12);
    xl_320_cfg.hw_send_byte(buffer[0]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[1]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[2]); // Note: this is not the same as AX-12
    xl_320_cfg.hw_send_byte(buffer[3]); //Reserved
    xl_320_cfg.hw_send_byte(buffer[4]); // XL-320 id
    xl_320_cfg.hw_send_byte(buffer[5]); // packet length
    xl_320_cfg.hw_send_byte(buffer[6]); // packet length MSB assuming length is <255
    xl_320_cfg.hw_send_byte(buffer[7]); // content (instruction or status)
    xl_320_cfg.hw_send_byte(buffer[8]); // param1
    xl_320_cfg.hw_send_byte(buffer[9]); // param2
    xl_320_cfg.hw_send_byte(buffer[10]); // param3
    xl_320_cfg.hw_send_byte(buffer[11]); // param4
    xl_320_cfg.hw_send_byte(crc16 & 0xFF); // CRC_L
    xl_320_cfg.hw_send_byte((crc16>>8)&0xFF); // CRC_H

    // Get back to RX mode
    xl_320_cfg.hw_switch(XL_320_RX);
  

}

unsigned int xl_320_set_goal_torque(unsigned char id, unsigned int goal_torque) {
    unsigned short crc16;
    uint8_t torque_LSB=goal_torque&0xFF;
    uint8_t torque_MSB=((goal_torque>>8)&0xFF);
    uint8_t buffer[]={XL_320_HEADER,XL_320_HEADER,XL_320_HEADER_2,0,id,0x07,0x00,0x03,XL_320_ADDR_GOAL_TORQUE_L,00,torque_LSB,torque_MSB};
    // Set uart2 half duplex to TX mode
    xl_320_cfg.hw_switch(XL_320_TX);
    crc16 = update_crc(0, buffer,12);
    xl_320_cfg.hw_send_byte(buffer[0]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[1]); // 8 bits header for sync
    xl_320_cfg.hw_send_byte(buffer[2]); // Note: this is not the same as AX-12
    xl_320_cfg.hw_send_byte(buffer[3]); //Reserved
    xl_320_cfg.hw_send_byte(buffer[4]); // XL-320 id
    xl_320_cfg.hw_send_byte(buffer[5]); // packet length
    xl_320_cfg.hw_send_byte(buffer[6]); // packet length MSB assuming length is <255
    xl_320_cfg.hw_send_byte(buffer[7]); // content (instruction or status)
    xl_320_cfg.hw_send_byte(buffer[8]); // param1
    xl_320_cfg.hw_send_byte(buffer[9]); // param2
    xl_320_cfg.hw_send_byte(buffer[10]); // param3
    xl_320_cfg.hw_send_byte(buffer[11]); // param4
    xl_320_cfg.hw_send_byte(crc16 & 0xFF); // CRC_L
    xl_320_cfg.hw_send_byte((crc16>>8)&0xFF); // CRC_H

    // Get back to RX mode
    xl_320_cfg.hw_switch(XL_320_RX);


    
}



void xl_320_send_packet(xl_320_packet_t* packet)
{
    unsigned char idx_param;

        // Set uart2 half duplex to TX mode
        xl_320_cfg.hw_switch(XL_320_TX);

        xl_320_cfg.hw_send_byte(XL_320_HEADER); // 8 bits header for sync
        xl_320_cfg.hw_send_byte(XL_320_HEADER); // 8 bits header for sync
        xl_320_cfg.hw_send_byte(XL_320_HEADER_2); // Note: this is not the same as AX-12
        xl_320_cfg.hw_send_byte(0x00); //Reserved
        xl_320_cfg.hw_send_byte(packet->id);       // XL-320 id
        xl_320_cfg.hw_send_byte(packet->length);   // packet length
        xl_320_cfg.hw_send_byte(0x00);   // packet length MSB assuming length is <255
        xl_320_cfg.hw_send_byte(packet->content);  // content (instruction or status)

        // Parameters
        if(packet->length > 3)
        {
            for(idx_param = 0; idx_param < packet->length-3 ; idx_param++)
            {
                xl_320_cfg.hw_send_byte(packet->parameters[idx_param]);
            }
        }

        // Checksum
        xl_320_cfg.hw_send_byte((uint8_t)(packet->checksum &0x00FF));    // Checksum LSB
        xl_320_cfg.hw_send_byte((uint8_t)(packet->checksum>>8) &0x00FF); // Checksum MSB
        // Get back to RX mode
        xl_320_cfg.hw_switch(XL_320_RX);

}

// Status check to be added
unsigned int xl_320_receive_packet(xl_320_packet_t* packet)
{
    unsigned char idx_param;
    unsigned char rx_buffer;
    unsigned char status;

    // Initialize values
    packet->id      = XL_320_ID_IMPOSSIBLE;   // Because nobody can have this ID
    packet->length  = 0;
    packet->content = 0;
    for(idx_param=0; idx_param<XL_320_MAX_PARAM; idx_param++)
        packet->parameters[idx_param] = 0;
    idx_param = 0;
    status = 0;

    // Flush RX before receiving anything
    xl_320_cfg.hw_flush();
    
    do {
        status = xl_320_cfg.hw_receive_byte(&rx_buffer);
    } while((rx_buffer != XL_320_HEADER) && (!(status&0x01)));

      if(status == 1)
        return XL_320_ERROR_UART_TIMEOUT;

    // Wait for the 1st header and retrieve it
    /*status = xl_320_cfg.hw_receive_byte(&rx_buffer);
    if(status == 1)
        return xl_320_ERROR_UART_TIMEOUT;
    if(rx_buffer != (unsigned char) xl_320_HEADER)
        return xl_320_ERROR_UART_HEADER;
      */
    // Wait for the 2nd header and retrieve it
    status |= xl_320_cfg.hw_receive_byte(&rx_buffer);
    if(rx_buffer != (unsigned char) XL_320_HEADER)
        return XL_320_ERROR_UART_HEADER;

    // Wait for the id and retrieve it
    status |= xl_320_cfg.hw_receive_byte(&rx_buffer);
    packet->id = rx_buffer;

    // Wait for the length and retrieve it
    status |= xl_320_cfg.hw_receive_byte(&rx_buffer);
    packet->length = rx_buffer;

    // Wait for the status and retrieve it
    status |= xl_320_cfg.hw_receive_byte(&rx_buffer);
    packet->content = rx_buffer;

    // Retrieve parameters, length must be greater than 2
    if(packet->length > 2)
        for(idx_param=0; idx_param<packet->length-2;idx_param++)
        {
            // Wait for the parameter and retrieve it
            status |= xl_320_cfg.hw_receive_byte(&rx_buffer);
            (packet->parameters)[idx_param] = rx_buffer;
        }

    // Wait for the checksum and retrieve it
    status |= xl_320_cfg.hw_receive_byte(&rx_buffer);
    packet->checksum = rx_buffer & 0xFF;

    return status;
}

uint16_t xl_320_compute_checksum(xl_320_packet_t* packet)
{
    uint8_t data_blk_ptr[XL_320_MAX_PARAM+5]={0};
    uint16_t i, j;
    uint16_t crc_accum;
    uint16_t crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };


    /*Fetch the whole frame and store it into a single buffer*/
    data_blk_ptr[0]=XL_320_HEADER;
    data_blk_ptr[1]=XL_320_HEADER;
    data_blk_ptr[2]=XL_320_HEADER_2;
    data_blk_ptr[3]=0; //Reserved
    data_blk_ptr[4]=packet->id;
    data_blk_ptr[5]=packet->length;
    data_blk_ptr[6]=0; //Assuming no packet length > 255
    data_blk_ptr[7]=packet->content;

    // Parameters
    if(packet->length > 3)
    {
        for(j = 0; j < packet->length-2; j++)
        {
            data_blk_ptr[j+8]=packet->parameters[j];
        }
    }

    for(j = 0; j < packet->length+5; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}

// -----------------------------------------------------------------------------
// INSTRUCTION layer: handles XL-320 instruction set
// -----------------------------------------------------------------------------

// Gather all status: from XL-320 but also from the UART layer
unsigned int xl_320_get_status(xl_320_packet_t* instruction_packet,
                                xl_320_packet_t* status_packet,
                                unsigned char expected_param_length)
{
    unsigned int status;
    
    // Retrieve hardware status from XL-320
    status = (status_packet->content) & 0xFF;
    
    // Check that the status packet id matches the sender
    if(status_packet->id != instruction_packet->id)
        status |= XL_320_ERROR_APP_ID;

    // Check that received parameter length is correct
    if(status_packet->length != expected_param_length+2)
        status |= XL_320_ERROR_APP_LENGTH;

    // Check that checksum is correct, this is not
    // revealant if length is incorrect
    if(status_packet->checksum != xl_320_compute_checksum(status_packet)) {
        status |= XL_320_ERROR_APP_CHECKSUM;
        //printf("Checksums : 0x%02X vs 0x%02X\n", status_packet->checksum ,xl_320_compute_checksum(status_packet)  );
    }

    //xl_320_print_packet(&status_packet);
    
    return status;
}

// Ping instruction
unsigned int xl_320_ping(unsigned char id)
{
    unsigned int status;
    xl_320_packet_t ping_packet;
    xl_320_packet_t status_packet;

    // Build the instruction packet
    ping_packet.id       = id;
    ping_packet.length   = 3; // 0 parameters
    ping_packet.content  = XL_320_INS_PING;
    ping_packet.checksum = xl_320_compute_checksum(&ping_packet);

    status = 0;
    
    // Send the ping instruction
    xl_320_send_packet(&ping_packet);

    // A status packet is always returned after a ping instruction.
    // However check that id was not broadcast, even if that's a dum operation.
    if((id !=  XL_320_ID_BROADCAST))
    {        
        // Retrieve a status packet, add the UART error flags
        status = xl_320_receive_packet(&status_packet);

        // Get the overall status, add the CDS55XX and APP error flags
        status |= xl_320_get_status(&ping_packet, &status_packet, 0);
    }

    return status;   
}

// Write instruction
// If registered parameter is set, it'll be a reg_write instruction.
unsigned int xl_320_write(unsigned char id, unsigned char address, unsigned char* parameters,
                           unsigned char nb_param, unsigned char registered)
{
    unsigned int status;
    unsigned char idx_param;
    xl_320_packet_t write_packet;
    xl_320_packet_t status_packet;

    // Build the packet
    write_packet.id            = id;
    write_packet.length        = 3+nb_param+2;
    write_packet.content       = registered?XL_320_INS_REG_WRITE:XL_320_INS_WRITE;
    write_packet.parameters[0] = address & 0x00FF; // LSW
    write_packet.parameters[1] = (address >> 8); // MSW

    for(idx_param=0; idx_param < nb_param; idx_param++)
    {
        write_packet.parameters[idx_param+2] = parameters[idx_param];
    }
    write_packet.checksum = xl_320_compute_checksum(&write_packet);

    // Send the instruction
    xl_320_send_packet(&write_packet);
    
    status = 0;

    // A status packet is returned only if address is not broadcast and
    // the status return level is set to "all packets".
    if((id != (unsigned char) XL_320_ID_BROADCAST) &&
        xl_320_cfg.return_level == XL_320_STATUS_EVERYTHING)
    {
        // Retrieve a status packet, add the UART error flags
        status = xl_320_receive_packet(&status_packet);

        // Get the overall status, add the CDS55XX and APP error flags
        status |= xl_320_get_status(&write_packet, &status_packet, 0);
    }

    return status;
}

// Read instruction
unsigned int xl_320_read(unsigned char id, unsigned char address, unsigned char* datas, unsigned char nb_data)
{
    unsigned int status;
    xl_320_packet_t read_packet;
    xl_320_packet_t status_packet;
    unsigned char idx_data;    

    // Build the packet
    read_packet.id            = id;
    read_packet.length        = 4;
    read_packet.content       = XL_320_INS_READ;
    read_packet.parameters[0] = address;
    read_packet.parameters[1] = nb_data;
    read_packet.checksum      = xl_320_compute_checksum(&read_packet);

    // Send the instruction
    xl_320_send_packet(&read_packet);

    status = 0;

    // A read status packet is returned only if address is not broadcast and
    // the status return level is different than "non".
    if((id != (unsigned char) XL_320_ID_BROADCAST) &&
        xl_320_cfg.return_level != XL_320_STATUS_NO_ANSWER)
    {
        // Retrieve a status packet, add the UART error flags
        status = xl_320_receive_packet(&status_packet);

        // Get the overall status, add the CDS55XX and APP error flags
        status |= xl_320_get_status(&read_packet, &status_packet, nb_data);

        // Affect read datas if no critical error happened during the
        // status packet reception
        if(!(status & 0x00FF))
            for(idx_data=0; idx_data<status_packet.length-2; idx_data++)
                datas[idx_data] = status_packet.parameters[idx_data];

    }

    //xl_320_print_packet(&status_packet);
    return status;
}

// Action instruction
void xl_320_action(void)
{
    xl_320_packet_t action_packet;
 
    // Build the packet
    action_packet.id        = XL_320_ID_BROADCAST;
    action_packet.length    = 2;
    action_packet.content   = XL_320_INS_ACTION;
    action_packet.checksum  = xl_320_compute_checksum(&action_packet);

    // Send the instruction
    xl_320_send_packet(&action_packet);
}

// Reset instruction: /!\ carrefull this will factory reset everything including baudrate and IDs
unsigned int xl_320_reset(unsigned char id)
{
    unsigned int status;
    xl_320_packet_t reset_packet;
    xl_320_packet_t status_packet;

    // Build the instruction packet
    reset_packet.id       = id;
    reset_packet.length   = 2; // 0 parameters
    reset_packet.content  = XL_320_INS_RESET;
    reset_packet.checksum = xl_320_compute_checksum(&reset_packet);

    // Send the instruction
    xl_320_send_packet(&reset_packet);

    // A status packet is returned only if address is not broadcast and
    // the status return level is set to "all packets".
    if((id != (unsigned char) XL_320_ID_BROADCAST) &&
        xl_320_cfg.return_level == XL_320_STATUS_EVERYTHING)
    {
        // Retrieve a status packet, add the UART error flags
        status = xl_320_receive_packet(&status_packet);

        // Get the overall status, add the CDS55XX and APP error flags
        status |= xl_320_get_status(&reset_packet, &status_packet, 0);
    }

    return status;
}

// Synchronized write instruction
void xl_320_sync_write(unsigned char* id, unsigned char address, unsigned char** parameters,
                        unsigned char nb_id, unsigned char nb_param)
{
    unsigned char idx_id;
    unsigned char idx_param;
    xl_320_packet_t sync_packet;
 
    // Build the packet
    sync_packet.id            = XL_320_ID_BROADCAST;
    sync_packet.length        = (nb_param + 1) * nb_id + 4;
    sync_packet.content       = XL_320_INS_WRITE;
    sync_packet.parameters[0] = address;
    sync_packet.parameters[1] = nb_param;

    for(idx_id=0; idx_id < nb_id; idx_id++)
    {
        sync_packet.parameters[idx_id+2] = id[idx_id];

        for(idx_param=0; idx_param < nb_param; idx_param++)
            sync_packet.parameters[idx_id*nb_param + idx_param] = parameters[idx_id][idx_param];

    }
    sync_packet.checksum = xl_320_compute_checksum(&sync_packet);

    // Send the instruction
    xl_320_send_packet(&sync_packet);

}

// -----------------------------------------------------------------------------
// Application layer: single access function set
// -----------------------------------------------------------------------------

//
// Shortcut  functions
//

 unsigned int xl_320_write_int(unsigned char id, unsigned char address, unsigned int data, unsigned char registered)
{
    unsigned char params[2];
    params[0] = data & 0x00FF; // LSW
    params[1] = (data >> 8);   // MSW
    return xl_320_write(id, address, params, 2, registered);
}

 unsigned int xl_320_read_int(unsigned char id, unsigned char address, unsigned int *data)
{
    unsigned char datas[2];
    unsigned int status;

    datas[0] = 0;
    datas[1] = 0;
    
    status = xl_320_read(id, address, datas, 2);
    //printf("%X %X\n",data[1]&0xFF, data[0]&0xFF);
    (*data) = ((unsigned int) datas[0]) + ((unsigned int) (datas[1]&0x03)<<8);
    return status;
}

//
// EEPROM set functions
// They all take effect immediatly (not registered).
//

// As we dont know the device's id, it is broadcast to everyone,
// so all connected devices will now have the requested id.
// unsigned int xl_320_set_id(unsigned char id) {
//    return xl_320_write(XL_320_ID_BROADCAST, XL_320_ADDR_ID, &id, 1, 0); }

// unsigned int xl_320_set_baudrate(unsigned char id, unsigned char baudrate) {
//    return xl_320_write(id, XL_320_ADDR_BAUD_RATE, &baudrate, 1, 0); }

 unsigned int xl_320_set_return_delay(unsigned char id, unsigned char return_delay) {
    return xl_320_write(id, XL_320_ADDR_RETURN_DELAY, &return_delay, 1, 0);
    xl_320_cfg.return_delay = return_delay; }

 unsigned int xl_320_set_temp_limit(unsigned char id, unsigned char temp_limit) {
    return xl_320_write(id, XL_320_ADDR_LIM_TEMP, &temp_limit, 1, 0); }

 unsigned int xl_320_set_min_voltage(unsigned char id, unsigned char min_voltage) {
    return xl_320_write(id, XL_320_ADDR_LIM_L_VOLTAGE, &min_voltage, 1, 0); }

 unsigned int xl_320_set_max_voltage(unsigned char id, unsigned char max_voltage) {
    return xl_320_write(id, XL_320_ADDR_LIM_H_VOLTAGE, &max_voltage, 1, 0); }

 //unsigned int xl_320_set_alarm_shutdown(unsigned char id, unsigned char alarm_shutdown) {
 //   return xl_320_write(id, XL_320_ADDR_ALARM_SHUTDOWN, &alarm_shutdown, 1, 0); }

 unsigned int xl_320_set_cw_limit(unsigned char id, unsigned int cw_limit) {
    return xl_320_write_int(id, XL_320_ADDR_CW_LIM_L, cw_limit, 0); }

 unsigned int xl_320_set_ccw_limit(unsigned char id, unsigned int ccw_limit) {
    return xl_320_write_int(id, XL_320_ADDR_CCW_LIM_L, ccw_limit, 0); }

// unsigned int xl_320_set_torque_limit(unsigned char id, unsigned int torque_limit) {
//    return xl_320_write_int(id, XL_320_ADDR_MAX_TORQUE_L, torque_limit, 0); }

 unsigned int xl_320_set_status_return(unsigned char id, unsigned char status_return) {
    xl_320_cfg.return_level = status_return;
    return xl_320_write(id, XL_320_ADDR_STATUS_RETURN, &status_return, 1, 0); }

//
// RAM set functions
// Revealant functions have a registered possibility; calling xl_320_action()
// is then necessarry to trigger any effect.
// Otherwise, they take effect immediatly.
//


// unsigned int xl_320_set_torque_en(unsigned char id, unsigned char torque_en, unsigned char registered) {
//    return xl_320_write(id, XL_320_ADDR_TORQUE_EN, &torque_en, 1, registered); }

// unsigned int xl_320_set_led(unsigned char id, unsigned char led) {
//    return xl_320_write(id, XL_320_ADDR_LED, &led, 1,0); }

// unsigned int xl_320_set_position(unsigned char id, unsigned int position, unsigned char registered) {
//    return xl_320_write_int(id, XL_320_ADDR_GOAL_POS_L, position, registered); }

 //unsigned int xl_320_set_speed(unsigned char id, unsigned int speed, unsigned char registered) {
 //   return xl_320_write_int(id, XL_320_ADDR_GOAL_VELOCITY_L , speed, registered); }

 unsigned int xl_320_set_punch(unsigned char id, unsigned int punch, unsigned char registered) {
    return xl_320_write_int(id, XL_320_ADDR_PUNCH_L, punch, registered); }


//
// RAM get functions
//

 unsigned int xl_320_get_temperature(unsigned char id, unsigned char *temp) {
    return xl_320_read(id, XL_320_ADDR_CUR_TEMP, temp, 1); }

 unsigned int xl_320_get_voltage(unsigned char id, unsigned char *voltage) {
    return xl_320_read(id, XL_320_ADDR_CUR_VOLT, voltage, 1); }

 unsigned int xl_320_get_moving(unsigned char id, unsigned char *moving) {
    return xl_320_read(id, XL_320_ADDR_MOVING, moving, 1); }

 unsigned int xl_320_get_position(unsigned char id, unsigned int *position) {
    return xl_320_read_int(id, XL_320_ADDR_CUR_POS_L, position); }

 unsigned int xl_320_get_speed(unsigned char id, unsigned int *speed) {
    return xl_320_read_int(id, XL_320_ADDR_CUR_SPEED_L, speed); }

 unsigned int xl_320_get_load(unsigned char id, unsigned int *load) {
    return xl_320_read_int(id, XL_320_ADDR_CUR_LOAD_L, load); }

// -----------------------------------------------------------------------------
// Application layer: burst access (multiple registers), synchronized motion
// -----------------------------------------------------------------------------

// Set the motion parameters of one servo but register the motion.
// Calling xl_320_action() is necessary to trigger the motion
 unsigned int xl_320_set_motion_reg(unsigned char id, unsigned int position,
                                           unsigned int speed, unsigned int torque)
{
    unsigned char params[6];
    params[0] = position & 0x00FF;
    params[1] = position >> 8;
    params[2] = speed & 0x00FF;
    params[3] = speed >> 8;
    params[4] = torque & 0x00FF;
    params[5] = torque >> 8;

    return xl_320_write(id, XL_320_ADDR_GOAL_POS_L, params, 6, 1);

}
// Set the motion parameters (goal, speed, acceleration) of an array of servos.
// This functions can handle a maximum of 16 servos.
 unsigned int xl_320_set_motions(unsigned char *ids, unsigned int *positions,
                    unsigned int *speeds, unsigned char *accs, unsigned char *dccs,
                    unsigned char nb_id)
{
    unsigned char idx_id;
    unsigned char params[16][6]; // 16 servos x 6 parameters
    unsigned int *cur_position;
    unsigned int *cur_speed;
    unsigned char *cur_acc;
    unsigned char *cur_dcc;

    // Init array pointers
    cur_position = positions;
    cur_speed = speeds;
    cur_acc = accs;
    cur_dcc = dccs;

    // Build the data table
    for(idx_id=0; idx_id<nb_id; idx_id++)
    {
        params[idx_id][0] = (*cur_position) & 0x00FF;
        params[idx_id][1] = (*cur_position) >> 8;
        params[idx_id][2] = (*cur_speed) & 0x00FF;
        params[idx_id][3] = (*cur_position) >> 8;
        params[idx_id][4] = (*cur_acc);
        params[idx_id][5] = (*cur_dcc);

        // Increment indexes
        cur_position++;
        cur_speed++;
        cur_acc++;
        cur_dcc++;
    }
    
    xl_320_sync_write(ids, XL_320_ADDR_GOAL_POS_L, (unsigned char**) params, nb_id, 6);

    // All packets were broadcasted
    return 0;
}

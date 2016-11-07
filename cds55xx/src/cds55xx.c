
#include "board_cfg.h"
#include "cds55xx.h"

// The CDS55XX must be declared as a global outside of this module.
// This is done in order to ease the manipulation of the different
// high-level functions (avoiding to give the current config as
// an argument).
extern cds55xx_cfg_t cds55xx_cfg;

// -----------------------------------------------------------------------------
// First layer: handles UART data
// -----------------------------------------------------------------------------

void cds55xx_send_packet(cds55xx_packet_t* packet)
{
    unsigned char idx_param;
    
        // Set uart2 half duplex to TX mode
        cds55xx_cfg.hw_switch(CDS55XX_TX);

        cds55xx_cfg.hw_send_byte(CDS55XX_HEADER); // 8 bits header for sync
        cds55xx_cfg.hw_send_byte(CDS55XX_HEADER); // 8 bits header for sync
        cds55xx_cfg.hw_send_byte(packet->id);       // cds55xx id
        cds55xx_cfg.hw_send_byte(packet->length);   // packet length
        cds55xx_cfg.hw_send_byte(packet->content);  // content (instruction or status)

        // Parameters
        if(packet->length > 2)
            for(idx_param = 0; idx_param < packet->length-2 ; idx_param++)
                cds55xx_cfg.hw_send_byte(packet->parameters[idx_param]);

        // Checksum
        cds55xx_cfg.hw_send_byte(packet->checksum);
IO_4=1;
        // Get back to RX mode
        cds55xx_cfg.hw_switch(CDS55XX_RX);
IO_4=0;
}

// Status check to be added
unsigned int cds55xx_receive_packet(cds55xx_packet_t* packet)
{
    unsigned char idx_param;
    unsigned char rx_buffer;
    unsigned char status;

    // Initialize values
    packet->id      = CDS55XX_ID_IMPOSSIBLE;   // Because nobody can have this ID
    packet->length  = 0;
    packet->content = 0;
    for(idx_param=0; idx_param<CDS55XX_MAX_PARAM; idx_param++)
        packet->parameters[idx_param] = 0;
    idx_param = 0;
    status = 0;

    // Flush RX before receiving anything
    cds55xx_cfg.hw_flush();
    
    do {
        status = cds55xx_cfg.hw_receive_byte(&rx_buffer);
    } while((rx_buffer != CDS55XX_HEADER) && (!(status&0x01)));

      if(status == 1)
        return CDS55XX_ERROR_UART_TIMEOUT;

    // Wait for the 1st header and retrieve it
    /*status = cds55xx_cfg.hw_receive_byte(&rx_buffer);
    if(status == 1)
        return CDS55XX_ERROR_UART_TIMEOUT;    
    if(rx_buffer != (unsigned char) CDS55XX_HEADER)
        return CDS55XX_ERROR_UART_HEADER;
      */
    // Wait for the 2nd header and retrieve it
    status |= cds55xx_cfg.hw_receive_byte(&rx_buffer);
    if(rx_buffer != (unsigned char) CDS55XX_HEADER)
        return CDS55XX_ERROR_UART_HEADER;

    // Wait for the id and retrieve it
    status |= cds55xx_cfg.hw_receive_byte(&rx_buffer);
    packet->id = rx_buffer;

    // Wait for the length and retrieve it
    status |= cds55xx_cfg.hw_receive_byte(&rx_buffer);
    packet->length = rx_buffer;

    // Wait for the status and retrieve it
    status |= cds55xx_cfg.hw_receive_byte(&rx_buffer);
    packet->content = rx_buffer;

    // Retrieve parameters, length must be greater than 2
    if(packet->length > 2)
        for(idx_param=0; idx_param<packet->length-2;idx_param++)
        {
            // Wait for the parameter and retrieve it
            status |= cds55xx_cfg.hw_receive_byte(&rx_buffer);
            (packet->parameters)[idx_param] = rx_buffer;
        }

    // Wait for the checksum and retrieve it
    status |= cds55xx_cfg.hw_receive_byte(&rx_buffer);
    packet->checksum = rx_buffer & 0xFF;

    return status;
}

unsigned char cds55xx_compute_checksum(cds55xx_packet_t* packet)
{
    unsigned char i;
    unsigned char checksum;

    // Init checksum with the 'static' values
    checksum = packet->id + packet->content + packet->length;

    // Add parameters value to checksum
    if(packet->length > 2)
        for(i=0; i<(packet->length)-2; i++)
            checksum += (packet->parameters)[i];

    return ~checksum;
}

// -----------------------------------------------------------------------------
// INSTRUCTION layer: handles CDS55XX instruction set
// -----------------------------------------------------------------------------

// Gather all status: from CDS55XX but also from the UART layer
unsigned int cds55xx_get_status(cds55xx_packet_t* instruction_packet,
                                cds55xx_packet_t* status_packet,
                                unsigned char expected_param_length)
{
    unsigned int status;
    
    // Retrieve hardware status from CDS55XX
    status = (status_packet->content) & 0xFF;
    
    // Check that the status packet id matches the sender
    if(status_packet->id != instruction_packet->id)
        status |= CDS55XX_ERROR_APP_ID;

    // Check that received parameter length is correct
    if(status_packet->length != expected_param_length+2)
        status |= CDS55XX_ERROR_APP_LENGTH;

    // Check that checksum is correct, this is not
    // revealant if length is incorrect
    if(status_packet->checksum != cds55xx_compute_checksum(status_packet)) {
        status |= CDS55XX_ERROR_APP_CHECKSUM;
        //printf("Checksums : 0x%02X vs 0x%02X\n", status_packet->checksum ,cds55xx_compute_checksum(status_packet)  );
    }

    //cds55xx_print_packet(&status_packet);
    
    return status;
}

// Ping instruction
unsigned int cds55xx_ping(unsigned char id)
{
    unsigned int status;
    cds55xx_packet_t ping_packet;
    cds55xx_packet_t status_packet;

    // Build the instruction packet
    ping_packet.id       = id;
    ping_packet.length   = 2; // 0 parameters
    ping_packet.content  = CDS55XX_INS_PING;
    ping_packet.checksum = cds55xx_compute_checksum(&ping_packet);

    status = 0;
    
    // Send the ping instruction
    cds55xx_send_packet(&ping_packet);

    // A status packet is always returned after a ping instruction.
    // However check that id was not broadcast, even if that's a dum operation.
    if((id !=  CDS55XX_ID_BROADCAST))
    {        
        // Retrieve a status packet, add the UART error flags
        status = cds55xx_receive_packet(&status_packet);

        // Get the overall status, add the CDS55XX and APP error flags
        status |= cds55xx_get_status(&ping_packet, &status_packet, 0);
    }

    return status;   
}

// Write instruction
// If registered parameter is set, it'll be a reg_write instruction.
unsigned int cds55xx_write(unsigned char id, unsigned char address, unsigned char* parameters,
                           unsigned char nb_param, unsigned char registered)
{
    unsigned int status;
    unsigned char idx_param;
    cds55xx_packet_t write_packet;
    cds55xx_packet_t status_packet;

    // Build the packet
    write_packet.id            = id;
    write_packet.length        = 3+nb_param;
    write_packet.content       = registered?CDS55XX_INS_REG_WRITE:CDS55XX_INS_WRITE;
    write_packet.parameters[0] = address;
    for(idx_param=0; idx_param < nb_param; idx_param++)
        write_packet.parameters[idx_param+1] = parameters[idx_param];
    write_packet.checksum = cds55xx_compute_checksum(&write_packet);

    // Send the instruction
    cds55xx_send_packet(&write_packet);
    
    status = 0;

    // A status packet is returned only if address is not broadcast and
    // the status return level is set to "all packets".
    if((id != (unsigned char) CDS55XX_ID_BROADCAST) &&
        cds55xx_cfg.return_level == CDS55XX_STATUS_EVERYTHING)
    {
        // Retrieve a status packet, add the UART error flags
        status = cds55xx_receive_packet(&status_packet);

        // Get the overall status, add the CDS55XX and APP error flags
        status |= cds55xx_get_status(&write_packet, &status_packet, 0);
    }

    return status;
}

// Read instruction
unsigned int cds55xx_read(unsigned char id, unsigned char address, unsigned char* datas, unsigned char nb_data)
{
    unsigned int status;
    cds55xx_packet_t read_packet;
    cds55xx_packet_t status_packet;
    unsigned char idx_data;
    unsigned int value;

    // Build the packet
    read_packet.id            = id;
    read_packet.length        = 4;
    read_packet.content       = CDS55XX_INS_READ;
    read_packet.parameters[0] = address;
    read_packet.parameters[1] = nb_data;
    read_packet.checksum      = cds55xx_compute_checksum(&read_packet);

    // Send the instruction
    cds55xx_send_packet(&read_packet);

    status = 0;

    // A read status packet is returned only if address is not broadcast and
    // the status return level is different than "non".
    if((id != (unsigned char) CDS55XX_ID_BROADCAST) &&
        cds55xx_cfg.return_level != CDS55XX_STATUS_NO_ANSWER)
    {
        // Retrieve a status packet, add the UART error flags
        status = cds55xx_receive_packet(&status_packet);

        // Get the overall status, add the CDS55XX and APP error flags
        status |= cds55xx_get_status(&read_packet, &status_packet, nb_data);

        // Affect read datas if no critical error happened during the
        // status packet reception
        if(!(status & 0x00FF))
            for(idx_data=0; idx_data<status_packet.length-2; idx_data++)
                datas[idx_data] = status_packet.parameters[idx_data];

    }

    //cds55xx_print_packet(&status_packet);
    return status;
}

// Action instruction
void cds55xx_action(void)
{
    cds55xx_packet_t action_packet;
 
    // Build the packet
    action_packet.id        = CDS55XX_ID_BROADCAST;
    action_packet.length    = 2;
    action_packet.content   = CDS55XX_INS_ACTION;
    action_packet.checksum  = cds55xx_compute_checksum(&action_packet);

    // Send the instruction
    cds55xx_send_packet(&action_packet);
}

// Reset instruction: /!\ carrefull this will factory reset everything including baudrate and IDs
unsigned int cds55xx_reset(unsigned char id)
{
    unsigned int status;
    cds55xx_packet_t reset_packet;
    cds55xx_packet_t status_packet;

    // Build the instruction packet
    reset_packet.id       = id;
    reset_packet.length   = 2; // 0 parameters
    reset_packet.content  = CDS55XX_INS_RESET;
    reset_packet.checksum = cds55xx_compute_checksum(&reset_packet);

    // Send the instruction
    cds55xx_send_packet(&reset_packet);

    // A status packet is returned only if address is not broadcast and
    // the status return level is set to "all packets".
    if((id != (unsigned char) CDS55XX_ID_BROADCAST) &&
        cds55xx_cfg.return_level == CDS55XX_STATUS_EVERYTHING)
    {
        // Retrieve a status packet, add the UART error flags
        status = cds55xx_receive_packet(&status_packet);

        // Get the overall status, add the CDS55XX and APP error flags
        status |= cds55xx_get_status(&reset_packet, &status_packet, 0);
    }

    return status;
}

// Synchronized write instruction
void cds55xx_sync_write(unsigned char* id, unsigned char address, unsigned char** parameters,
                        unsigned char nb_id, unsigned char nb_param)
{
    unsigned char idx_id;
    unsigned char idx_param;
    cds55xx_packet_t sync_packet;
 
    // Build the packet
    sync_packet.id            = CDS55XX_ID_BROADCAST;
    sync_packet.length        = (nb_param + 1) * nb_id + 4;
    sync_packet.content       = CDS55XX_INS_WRITE;
    sync_packet.parameters[0] = address;
    sync_packet.parameters[1] = nb_param;

    for(idx_id=0; idx_id < nb_id; idx_id++)
    {
        sync_packet.parameters[idx_id+2] = id[idx_id];

        for(idx_param=0; idx_param < nb_param; idx_param++)
            sync_packet.parameters[idx_id*nb_param + idx_param] = parameters[idx_id][idx_param];

    }
    sync_packet.checksum = cds55xx_compute_checksum(&sync_packet);

    // Send the instruction
    cds55xx_send_packet(&sync_packet);

}

// -----------------------------------------------------------------------------
// Application layer: single access function set
// -----------------------------------------------------------------------------

//
// Shortcut inline functions
//

inline unsigned int cds55xx_write_int(unsigned char id, unsigned char address, unsigned int data, unsigned char registered)
{
    unsigned char params[2];
    params[0] = data & 0x00FF; // LSW
    params[1] = (data >> 8);   // MSW
    return cds55xx_write(id, address, params, 2, registered);
}

inline unsigned int cds55xx_read_int(unsigned char id, unsigned char address, unsigned int *data)
{
    unsigned char datas[2];
    unsigned int status;

    datas[0] = 0;
    datas[1] = 0;
    
    status = cds55xx_read(id, address, datas, 2);
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
inline unsigned int cds55xx_set_id(unsigned char id) {
    return cds55xx_write(CDS55XX_ID_BROADCAST, CDS55XX_ADDR_ID, &id, 1, 0); }

inline unsigned int cds55xx_set_baudrate(unsigned char id, unsigned char baudrate) {
    return cds55xx_write(id, CDS55XX_ADDR_BAUD_RATE, &baudrate, 1, 0); }

inline unsigned int cds55xx_set_return_delay(unsigned char id, unsigned char return_delay) {
    return cds55xx_write(id, CDS55XX_ADDR_RETURN_DELAY, &return_delay, 1, 0);
    cds55xx_cfg.return_delay = return_delay; }

inline unsigned int cds55xx_set_temp_limit(unsigned char id, unsigned char temp_limit) {
    return cds55xx_write(id, CDS55XX_ADDR_LIM_TEMP, &temp_limit, 1, 0); }

inline unsigned int cds55xx_set_min_voltage(unsigned char id, unsigned char min_voltage) {
    return cds55xx_write(id, CDS55XX_ADDR_LIM_L_VOLTAGE, &min_voltage, 1, 0); }

inline unsigned int cds55xx_set_max_voltage(unsigned char id, unsigned char max_voltage) {
    return cds55xx_write(id, CDS55XX_ADDR_LIM_H_VOLTAGE, &max_voltage, 1, 0); }

inline unsigned int cds55xx_set_alarm_led(unsigned char id, unsigned char alarm_led) {
    return cds55xx_write(id, CDS55XX_ADDR_ALARM_LED, &alarm_led, 1, 0); }

inline unsigned int cds55xx_set_alarm_shutdown(unsigned char id, unsigned char alarm_shutdown) {
    return cds55xx_write(id, CDS55XX_ADDR_ALARM_SHUTDOWN, &alarm_shutdown, 1, 0); }

inline unsigned int cds55xx_set_cw_limit(unsigned char id, unsigned int cw_limit) {
    return cds55xx_write_int(id, CDS55XX_ADDR_CW_LIM_L, cw_limit, 0); }

inline unsigned int cds55xx_set_ccw_limit(unsigned char id, unsigned int ccw_limit) {
    return cds55xx_write_int(id, CDS55XX_ADDR_CCW_LIM_L, ccw_limit, 0); }

inline unsigned int cds55xx_set_torque_limit(unsigned char id, unsigned int torque_limit) {
    return cds55xx_write_int(id, CDS55XX_ADDR_MAX_TORQUE_L, torque_limit, 0); }

inline unsigned int cds55xx_set_status_return(unsigned char id, unsigned char status_return) {
    return cds55xx_write(id, CDS55XX_ADDR_STATUS_RETURN, &status_return, 1, 0);
    cds55xx_cfg.return_level = status_return; }

//
// RAM set functions
// Revealant functions have a registered possibility; calling cds55xx_action()
// is then necessarry to trigger any effect.
// Otherwise, they take effect immediatly.
//

inline unsigned int cds55xx_set_cw_compliance(unsigned char id, unsigned char cw_compliance) {
    return cds55xx_write(id, CDS55XX_ADDR_CW_COMP_MARGIN, &cw_compliance, 1, 0); }

inline unsigned int cds55xx_set_ccw_compliance(unsigned char id, unsigned char ccw_compliance) {
    return cds55xx_write(id, CDS55XX_ADDR_CW_COMP_MARGIN, &ccw_compliance, 1, 0); }

inline unsigned int cds55xx_set_cw_proportion(unsigned char id, unsigned char cw_proportion) {
    return cds55xx_write(id, CDS55XX_ADDR_CW_COMP_MARGIN, &cw_proportion, 1, 0); }

inline unsigned int cds55xx_set_ccw_proportion(unsigned char id, unsigned char ccw_proportion) {
    return cds55xx_write(id, CDS55XX_ADDR_CW_COMP_MARGIN, &ccw_proportion, 1, 0); }

inline unsigned int cds55xx_set_acceleration(unsigned char id, unsigned char acceleration, unsigned char registered) {
    return cds55xx_write(id, CDS55XX_ADDR_ACC, &acceleration, 1, registered); }

inline unsigned int cds55xx_set_decceleration(unsigned char id, unsigned char decceleration, unsigned char registered) {
    return cds55xx_write(id, CDS55XX_ADDR_DCC, &decceleration, 1, registered); }

inline unsigned int cds55xx_set_torque_en(unsigned char id, unsigned char torque_en, unsigned char registered) {
    return cds55xx_write(id, CDS55XX_ADDR_TORQUE_EN, &torque_en, 1, registered); }

inline unsigned int cds55xx_set_led(unsigned char id, unsigned char led) {
    return cds55xx_write(id, CDS55XX_ADDR_LED, &led, 1, 0); }

inline unsigned int cds55xx_set_position(unsigned char id, unsigned int position, unsigned char registered) {
    return cds55xx_write_int(id, CDS55XX_ADDR_GOAL_POS_L, position, registered); }

inline unsigned int cds55xx_set_speed(unsigned char id, unsigned int speed, unsigned char registered) {
    return cds55xx_write_int(id, CDS55XX_ADDR_SPEED_L, speed, registered); }

inline unsigned int cds55xx_set_punch(unsigned char id, unsigned int punch, unsigned char registered) {
    return cds55xx_write_int(id, CDS55XX_ADDR_PUNCH_L, punch, registered); }

inline unsigned int cds55xx_set_lock(unsigned char id, unsigned char lock) {
    return cds55xx_write(id, CDS55XX_ADDR_LOCK, &lock, 1, 0); }
//
// EEPROM get functions
//

inline unsigned int cds55xx_get_model(unsigned char id, unsigned char *model) {
    return cds55xx_read(id, CDS55XX_ADDR_MODEL, model, 1); }

inline unsigned int cds55xx_get_firmware(unsigned char id, unsigned char *firmware) {
    return cds55xx_read(id, CDS55XX_ADDR_FIRMWARE, firmware, 1); }

inline unsigned int cds55xx_get_down_calibration(unsigned char id, unsigned int *down_calibration) {
    return cds55xx_read_int(id, CDS55XX_ADDR_DOWN_CAL_L, down_calibration); }

inline unsigned int cds55xx_get_up_calibration(unsigned char id, unsigned int *up_calibration) {
    return cds55xx_read_int(id, CDS55XX_ADDR_UP_CAL_L, up_calibration); }
//
// RAM get functions
//

inline unsigned int cds55xx_get_temperature(unsigned char id, unsigned char *temp) {
    return cds55xx_read(id, CDS55XX_ADDR_CUR_TEMP, temp, 1); }

inline unsigned int cds55xx_get_voltage(unsigned char id, unsigned char *voltage) {
    return cds55xx_read(id, CDS55XX_ADDR_CUR_VOLT, voltage, 1); }

inline unsigned int cds55xx_get_moving(unsigned char id, unsigned char *moving) {
    return cds55xx_read(id, CDS55XX_ADDR_MOVING, moving, 1); }

inline unsigned int cds55xx_get_position(unsigned char id, unsigned int *position) {
    return cds55xx_read_int(id, CDS55XX_ADDR_CUR_POS_L, position); }

inline unsigned int cds55xx_get_speed(unsigned char id, unsigned int *speed) {
    return cds55xx_read_int(id, CDS55XX_ADDR_CUR_SPEED_L, speed); }

inline unsigned int cds55xx_get_load(unsigned char id, unsigned int *load) {
    return cds55xx_read_int(id, CDS55XX_ADDR_CUR_LOAD_L, load); }

// -----------------------------------------------------------------------------
// Application layer: burst access (multiple registers), synchronized motion
// -----------------------------------------------------------------------------

// Set the motion parameters of one servo but register the motion.
// Calling cds55xx_action() is necessary to trigger.
inline unsigned int cds55xx_set_motion_reg(unsigned char id, unsigned int position,
                                           unsigned int speed, unsigned char acc, unsigned char dcc)
{
    unsigned char params[6];
    params[0] = position & 0x00FF;
    params[1] = position >> 8;
    params[2] = speed & 0x00FF;
    params[3] = speed >> 8;
    params[4] = acc;
    params[5] = dcc;

    return cds55xx_write(id, CDS55XX_ADDR_GOAL_POS_L, params, 6, 1);

}
// Set the motion parameters (goal, speed, acceleration) of an array of servos.
// This functions can handle a maximum of 16 servos.
inline unsigned int cds55xx_set_motions(unsigned char *ids, unsigned int *positions,
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
    
    cds55xx_sync_write(ids, CDS55XX_ADDR_GOAL_POS_L, (unsigned char**) params, nb_id, 6);

    // All packets were broadcasted
    return 0;
}

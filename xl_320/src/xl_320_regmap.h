/* 
 * File:   xl_320_regmap.h
 * Author: Sebastien Brulais
 *
 * Created on 15 octobre 2014, 11:30
 */

#ifndef XL_320_REGMAP_H
#define	XL_320_REGMAP_H


// -----------------------------------------------------------------------------
// Register map
// -----------------------------------------------------------------------------

// EEPROM Registers addresses
#define XL_320_ADDR_MODEL             0x01 // R
#define XL_320_ADDR_FIRMWARE          0x02 // R
#define XL_320_ADDR_ID                0x03 // RW
#define XL_320_ADDR_BAUD_RATE         0x04 // RW
#define XL_320_ADDR_RETURN_DELAY      0x05 // RW
#define XL_320_ADDR_CW_LIM_L          0x06 // RW
#define XL_320_ADDR_CW_LIM_H          0x07 // RW
#define XL_320_ADDR_CCW_LIM_L         0x08 // RW
#define XL_320_ADDR_CCW_LIM_H         0x09 // RW
#define XL_320_ADDR_CONTROL_MODE      0x0B
#define XL_320_ADDR_LIM_TEMP          0x0C // RW
#define XL_320_ADDR_LIM_L_VOLTAGE     0x0D // RW
#define XL_320_ADDR_LIM_H_VOLTAGE     0x0E // RW
#define XL_320_ADDR_MAX_TORQUE_L      0x0F // RW
#define XL_320_ADDR_MAX_TORQUE_H      0x10 // RW
#define XL_320_ADDR_STATUS_RETURN     0x11 // RW
#define XL_320_ADDR_ALARM_SHUTDOWN    0x12 // RW

// RAM Registers addresses
#define XL_320_ADDR_TORQUE_EN         0x18 // RW
#define XL_320_ADDR_LED               0x19 // RW
#define XL_320_ADDR_D_GAIN            0x1B // RW
#define XL_320_ADDR_I_GAIN            0x1C // RW
#define XL_320_ADDR_P_GAIN            0x1D // RW
#define XL_320_ADDR_GOAL_POS_L        0x1E // RW
#define XL_320_ADDR_GOAL_POS_H        0x1F // RW
#define XL_320_ADDR_GOAL_VELOCITY_L   0x20 // RW
#define XL_320_ADDR_GOAL_VELOCITY_H   0x21 // RW
#define XL_320_ADDR_GOAL_TORQUE_L     0x22 // RW
#define XL_320_ADDR_GOAL_TORQUE_H     0x23 // RW
#define XL_320_ADDR_CUR_POS_L         0x24 // R
#define XL_320_ADDR_CUR_POS_H         0x25 // R
#define XL_320_ADDR_CUR_SPEED_L       0x26 // R
#define XL_320_ADDR_CUR_SPEED_H       0x27 // R
#define XL_320_ADDR_CUR_LOAD_L        0x28 // R
#define XL_320_ADDR_CUR_LOAD_H        0x29 // R
#define XL_320_ADDR_CUR_VOLT          0x2D // R
#define XL_320_ADDR_CUR_TEMP          0x2E // R
#define XL_320_ADDR_REG_INSTR         0x2F // R
#define XL_320_ADDR_MOVING            0x31 // R
#define XL_320_ADDR_HW_ERR            0x32 // R
#define XL_320_ADDR_PUNCH_L           0x33 // RW
#define XL_320_ADDR_PUNCH_H           0x34 // RW



#endif	/* XL_320_REGMAP_H */


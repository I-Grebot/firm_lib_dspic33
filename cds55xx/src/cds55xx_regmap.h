#ifndef _CDS55XX_REGMAP_H
#define _CDS55XX_REGMAP_H

// -----------------------------------------------------------------------------
// Register map
// -----------------------------------------------------------------------------

// EEPROM Registers addresses
#define CDS55XX_ADDR_MODEL             0x01 // R
#define CDS55XX_ADDR_FIRMWARE          0x02 // R
#define CDS55XX_ADDR_ID                0x03 // RW
#define CDS55XX_ADDR_BAUD_RATE         0x04 // RW
#define CDS55XX_ADDR_RETURN_DELAY      0x05 // RW
#define CDS55XX_ADDR_CW_LIM_L          0x06 // RW
#define CDS55XX_ADDR_CW_LIM_H          0x07 // RW
#define CDS55XX_ADDR_CCW_LIM_L         0x08 // RW
#define CDS55XX_ADDR_CCW_LIM_H         0x09 // RW
#define CDS55XX_ADDR_LIM_TEMP          0x0B // RW
#define CDS55XX_ADDR_LIM_L_VOLTAGE     0x0C // RW
#define CDS55XX_ADDR_LIM_H_VOLTAGE     0x0D // RW
#define CDS55XX_ADDR_MAX_TORQUE_L      0x0E // RW
#define CDS55XX_ADDR_MAX_TORQUE_H      0x0F // RW
#define CDS55XX_ADDR_STATUS_RETURN     0x10 // RW
#define CDS55XX_ADDR_ALARM_LED         0x11 // RW
#define CDS55XX_ADDR_ALARM_SHUTDOWN    0x12 // RW
#define CDS55XX_ADDR_DOWN_CAL_L        0x14 // R
#define CDS55XX_ADDR_DOWN_CAL_H        0x15 // R
#define CDS55XX_ADDR_UP_CAL_L          0x16 // R
#define CDS55XX_ADDR_UP_CAL_H          0x17 // R

// RAM Registers addresses
#define CDS55XX_ADDR_TORQUE_EN         0x18 // RW
#define CDS55XX_ADDR_LED               0x19 // RW
#define CDS55XX_ADDR_CW_COMP_MARGIN    0x1A // RW
#define CDS55XX_ADDR_CCW_COMP_MARGIN   0x1B // RW
#define CDS55XX_ADDR_CW_PROPORTION     0x1C // RW
#define CDS55XX_ADDR_CCW_PROPORTION    0x1D // RW
#define CDS55XX_ADDR_GOAL_POS_L        0x1E // RW
#define CDS55XX_ADDR_GOAL_POS_H        0x1F // RW
#define CDS55XX_ADDR_SPEED_L           0x20 // RW
#define CDS55XX_ADDR_SPEED_H           0x21 // RW
#define CDS55XX_ADDR_ACC               0x22 // RW
#define CDS55XX_ADDR_DCC               0x23 // RW
#define CDS55XX_ADDR_CUR_POS_L         0x24 // R
#define CDS55XX_ADDR_CUR_POS_H         0x25 // R
#define CDS55XX_ADDR_CUR_SPEED_L       0x26 // R
#define CDS55XX_ADDR_CUR_SPEED_H       0x27 // R
#define CDS55XX_ADDR_CUR_LOAD_L        0x28 // R
#define CDS55XX_ADDR_CUR_LOAD_H        0x29 // R
#define CDS55XX_ADDR_CUR_VOLT          0x2A // R
#define CDS55XX_ADDR_CUR_TEMP          0x2B // R
#define CDS55XX_ADDR_REG_INSTR         0x2C // RW
#define CDS55XX_ADDR_MOVING            0x2E // RW
#define CDS55XX_ADDR_LOCK              0x2F // RW (once)
#define CDS55XX_ADDR_PUNCH_L           0x30 // RW
#define CDS55XX_ADDR_PUNCH_H           0x31 // RW

#endif // _CDS55XX_H



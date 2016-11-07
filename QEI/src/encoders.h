#ifndef _ENCODERS_H_
#define _ENCODERS_H_

#include "board_cfg.h"

/** 
 * Initialisation of encoders, variables
 */
void encoders_init(void);



/** Extract encoder value.
 *
 * \param data : a (void *) that is casted in (uint8_t) containing the number
 *               of the encoder to be read.
 */
sint32_t encoders_get_value(uint8_t data);

/** Set an encoder value
 *
 * \param data : a (void *) that is casted in (uint8_t) containing the number
 *               of the encoder to be read.
 * \param v    : the value
 */
void encoders_set_value(void * data, sint32_t c);

#endif

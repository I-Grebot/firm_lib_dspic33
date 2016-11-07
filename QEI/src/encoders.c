/** \file encoders.c
 *  \brief Implementation for getting motor position
 *
 */

#include <encoders.h>

static sint32_t value_codeur[2];
static sint16_t value_old[2];

/** 
 * Initialisation of encoders, variables
 */
void encoders_init(void)
{
	QEI1CONbits.QEIM=0b111;     // signal x4 + overflow reset by MAXxCNT
	MAX1CNT=0xFFFF;
	QEI1CONbits.SWPAB=0;        // counter unswaped
	QEI1CONbits.PCDOUT=0;       // no UPDNx output

//	DFLT1CONbits.QEOUT=1;       // Enable digital filter
//	DFLT1CONbits.QECK=0;        // clock divided 1:1

	QEI2CON=QEI1CON;            // same configuration for both encoders
	MAX2CNT=MAX1CNT;
	//DFLT2CON=DFLT1CON;

    encoders_set_value((void*)0,0);
    encoders_set_value((void*)1,0);
}


/** Extract encoder value.
 *
 * \param number : a (void *) that is casted in (int) containing the id
 *                 of the encoder to be read.
 */
sint32_t encoders_get_value(uint8_t id) {
    sint16_t value;
    sint32_t res;
    switch (id) {
        case 0:
            value = (sint16_t) POS1CNT;
            break;
        case 1:
            value = (sint16_t) POS2CNT;
            break;
        default:
            return 0;
    }
    res = (sint32_t)(value - value_old[id]);
    value_old[id] = value;

    value_codeur[id] += res;
    return value_codeur[id];
}

/** Set an encoder value
 *
 * \param number : a (void *) that is casted in (int) containing the number
 *                 of the encoder to be set.
 * \param v      : the value
 */
void encoders_set_value(void * number, sint32_t v)
{
	int num = (int) number;
	int val = (int) v;
	
	value_codeur[num]=val;
	value_old[num]=0;

	if (num == 0)
		POS1CNT=0;
	else 
		POS2CNT=0;
}

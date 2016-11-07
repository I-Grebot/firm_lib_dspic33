/* -----------------------------------------------------------------------------
 * IIR_c
 * I-Grebot IIR filter
 * -----------------------------------------------------------------------------
 * File        : IIR.c
 * Language    : C
 * Author      : Sebastien Brulais
 * Date        : 2013-05-02
 * -----------------------------------------------------------------------------
 * Description
 *   Simple 2nd order IIR filter computed with integer to reduce cpu operations
 *   When Fs and Fc are too close, use floats or fixed point instead
 *   More informations on http://wiki.igrebot.fr
 * -----------------------------------------------------------------------------
 * Versionning informations
 * Repository: http://svn2.assembla.com/svn/paranoid_android/
 * -----------------------------------------------------------------------------
 * $Rev:: 1170                                                                 $
 * $LastChangedBy:: sebastien.brulais                                          $
 * $LastChangedDate:: 2014-10-31 12:12:12 +0100 (ven., 31 oct. 2014)           $
 * -----------------------------------------------------------------------------
 * Version     Comment                                   Author       Date
 * 1.0         Initial release                           Seb B.      2013-05-02
 * ----------------------------------------------------------------------------- 
*/


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

// Filter constants generated by Matlab fdatool toolbox
// Butterworth 2nd order lowpass filter
// Fs=100Hz (user defined)
// b0, b1, b2 are normalized and doesn't change with Fc
static const int32_t b0=1;
static const int32_t b1=2;
static const int32_t b2=1;

// a0, a1 and a2 are devided by the gain given by fdatool

// Fc=1Hz, Fs=100hz
//////////////////////
static const int32_t a0=1059;
static const int32_t a1=-2023;
static const int32_t a2=968;
////////////////////////

// Fc=2Hz, Fs=100hz
//////////////////////
//static const int32_t a0=276;
//static const int32_t a1=-503;
//static const int32_t a2=231;
////////////////////////

// Fc=5Hz, Fs=100hz
//////////////////////
//static const int32_t a0=50;
//static const int32_t a1=-78;
//static const int32_t a2=32;
////////////////////////

// Fc=10Hz, Fs=100hz
//////////////////////
//static const int32_t a0=15;
//static const int32_t a1=-17;
//static const int32_t a2=6;
////////////////////////



//IIR specific variables
static int32_t x=0,x1=0,x2=0,y=0,y1=0,y2=0;

//------------------------------------------------------------------------------
// LOCAL FUNCTIONS
//------------------------------------------------------------------------------
int8_t s8_IIR(int8_t input_sample)
{
    
    x=(int32_t)(input_sample);
	//x*=10000; //input_sample should be rescale to avoid round issue when dividing
    y = b0*x;
    y +=b1*x1;
    y +=b2*x2;
    y -=a1*y1;
    y -=a2*y2;

    output_sample = y/a0;
    y2 = y1;
    y1 = output_sample;
    x2 = x1;
    x1 = x;
    y=0;
}




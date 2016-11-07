/* -----------------------------------------------------------------------------
 * Analog Color definition for DAC / ADC interfacing
 * I-Grebot
 * -----------------------------------------------------------------------------
 * File        : analog_color.h
 * Language    : C
 * Author      : Paul M.
 * Creation    : 2013-04-21
 * -----------------------------------------------------------------------------
 * Description
 *   This simple header contains color interface informations for Igrecolor
 *   sensor and other boards interfaced with it.
 *   It defines analog ranges so the color scales are correct.
 * -----------------------------------------------------------------------------
 * Versionning informations
 * Repository: http://svn2.assembla.com/svn/paranoid_android/
 * -----------------------------------------------------------------------------
 * $Rev:: 1170                                                                 $
 * $LastChangedBy:: sebastien.brulais                                          $
 * $LastChangedDate:: 2014-10-31 12:12:12 +0100 (ven., 31 oct. 2014)           $
 * -----------------------------------------------------------------------------
 * Version     Comment                                   Author       Date
 * 1.0         Initial release                           Paul M.      2013-04-21
 * -----------------------------------------------------------------------------
 */

#ifndef _ANALOG_COLOR_H_
#define _ANALOG_COLOR_H_

// -----------------------------------------------------------------------------
// 5 BIT (32 LEVELS) ADC/DAC CONFIGURATION
// -----------------------------------------------------------------------------

// Nominal values                5V Scale  3.3V Scale
#define ANA_COLOR_5B_BLACK    1 //  156     103
#define ANA_COLOR_5B_MAGENTA  5 //  781     516
#define ANA_COLOR_5B_BLUE     9 // 1406     928
#define ANA_COLOR_5B_CYAN    13 // 2031    1341
#define ANA_COLOR_5B_GREEN   17 // 2656    1753
#define ANA_COLOR_5B_YELLOW  21 // 3281    2166
#define ANA_COLOR_5B_RED     24 // 3906    2578
#define ANA_COLOR_5B_WHITE   29 // 4531    2991

// Margin around nominal values (tolerance)
#define ANA_COLOR_5B_MARGIN   1  // +/- 156mV @5V    +/- 103mV @3.3V

// -----------------------------------------------------------------------------
// 12 BIT (4096 LEVELS) ADC/DAC CONFIGURATION
// -----------------------------------------------------------------------------

// Nominal values                
#define ANA_COLOR_12B_BLACK     128
#define ANA_COLOR_12B_MAGENTA   640
#define ANA_COLOR_12B_BLUE     1152
#define ANA_COLOR_12B_CYAN     1664
#define ANA_COLOR_12B_GREEN    2176
#define ANA_COLOR_12B_YELLOW   2688
#define ANA_COLOR_12B_RED      3200
#define ANA_COLOR_12B_WHITE    3712

// Margin around nominal values (tolerance)
#define ANA_COLOR_12B_MARGIN    128


#endif // _ANALOG_COLOR_H_


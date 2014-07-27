//
//  Teensy3Servo.h
//
//  Created by Richard Nash on 7/13/14.
//  Copyright (c) 2014 Richard Nash.
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#ifndef __Teensy3Servo__
#define __Teensy3Servo__

#include <inttypes.h>

// The idea behind this library is to do hardware (FlexTimer) based servo signal generation
// with minimal storage required.
// Utilize the available Teensy3 PWM pins for servo control
// Note, this class has only static methods and memory, there is no need to instantiate it.
// Pins available are: 3, 4, 5, 6, 9, 10, 20, 21, 22, 23, 25, 32
// This corresponds to K20's timers:
// Teensy Pin       K20 Pin     Pin Name        Timer
//      3           28          PTA12(ALT3)     FTM1_CH0
//      4           29          PTA13(ALT3)     FTM1_CH1
//      5           64          PTD7(ALT4)      FTM0_CH7
//      6           61          PTD4(ALT4)      FTM0_CH4
//      9           46          PTC3(ALT4)      FTM0_CH2
//      10          49          PTC4(ALT4)      FTM0_CH3
//      20          62          PTD5(ALT4)      FTM0_CH5
//      21          63          PTD6(ALT4)      FTM0_CH6
//      22          44          PTC1(ALT4)      FTM0_CH0
//      23          45          PTC2(ALT4)      FTM0_CH1
//      25          42          PTB19(ALT3)     FTM2_CH1
//      32          41          PTB18(ALT3)     FTM2_CH0

/*
 * To use it, call "InitOut" with the pin you want to use to control the servo motor.
 * Note that this function completely consumes the given Timer (0, 1, or 2) so other
 * libraries that use Timers may interfere with it.
 *
 * Then just call Set(pin,value) with the value you want to set in degrees.
 *
 * This code assumes that the total cycle time is 20ms and that
 * 0 degrees is 1.5ms, -180 is 0.5ms, and +180 is 2.5ms
 * Your servo motors might be different.
 * Warning: Many servo motors will not travel the full -180 to +180
 * distance. Please check your servo's spec, before you burn it out by
 * running it into its stops.
 *
 * Example to sweep slowly back and forth using pin 3 (Timer 1, channel 0):
 *    Teensy3Servo::InitOut(3);
 *    while (1) {
 *      for (int i = -180; i < 180; i++) {
 *        Teensy3Servo::Set(3,i);
 *        delay(50);
 *      }
 *      for (int i = 180; i > -180; i--) {
 *        Teensy3Servo::Set(3,i);
 *        delay(50);
 *     }
 *   }
 */

class Teensy3Servo
{
public:
    static bool InitOut(int8_t teensyPin);
    static bool Set(int8_t teensyPin, int16_t val); // -180 < 0 < 180

    // Some day I hope to extend this to read the pulse widths as an input
    // For now these two functions don't do anything.
    static bool InitIn(int8_t teensyPin);
    static int16_t Get(int8_t teensyPin);
    
    // Counts public so that ISRs can get at them
    static uint16_t unmatchedRiseCounts[12];
    static uint16_t pulseWidth[12];
private:
    // Initialize output for FlexTimer0
    static void InitTimer(volatile uint32_t *FTMx_MODE,
                          volatile uint32_t *FTMx_COMBINE,
                          volatile uint32_t *FTMx_QDCTRL,
                          volatile uint32_t *FTMx_SC,
                          volatile uint32_t *FTMx_CNT,
                          volatile uint32_t *FTMx_MOD,
                          volatile uint32_t *FTMx_CNTIN);
    static bool InitByPin(uint8_t teensyPin, volatile uint32_t **channelControlRegister,
                          volatile uint32_t **pportControlRegister, uint32_t *paltNumber);
};

#endif /* defined(__Teensy3Servo__) */
